"""Independent L1-only A* topology search derived from frozen V5-r11.

The inherited map/semantic constructor and output contract are unchanged. The
first route uses admissible A* and visited-set dominance. Alternative routes
use a separate, unpruned A* pass so dominance cannot discard distinct routes.
"""

import heapq
import itertools
import math
import os
import time
from collections import OrderedDict, defaultdict
from dataclasses import dataclass

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

from arena_3d_v1.multires_topology import TopologyRoute
from arena_evaluation.semantic_map import canonical_hash
from .topology import V5Topology, join_selected_road_lines, right_line


@dataclass(slots=True)
class _Node:
    rid: int | None
    previous: int | None
    entry: tuple[float, float]
    visited: int
    pair: int
    parent: int
    piece: tuple | None
    cost: float


class AStarTopology(V5Topology):
    SCHEMA = "3YD-L1-AStar-v1"

    def __init__(self, world, graph_path, cache_root):
        super().__init__(world, graph_path, cache_root)
        # This belongs to initialization, so every query uses the same contact
        # geometry without repeatedly averaging long shared outlines.
        self.contact_centers = {
            key: tuple(map(float, np.asarray(points, float).mean(axis=0)))
            for key, points in self.contacts.items() if points
        }
        # Only the small first-phase certificate is retained. The large A*
        # frontier is never kept between queries. A failed first candidate can
        # then continue with the original alternate search without rerunning
        # the first A* pass.
        self._first_phase_cache = OrderedDict()
        self._last_first_phase_resume = False
        # Search transitions between two region contacts are static. Compile
        # them once at startup; start/goal transitions still use their actual
        # query positions and endpoint-specific traffic rules.
        self._static_transitions = {}
        if os.environ.get("THREEYD_V6_TRANSITION_TABLE", "0") == "1":
            for rid, neighbors in self.adj.items():
                kind = self.regions[rid]["kind"]
                axis = self.router.axes[rid] if kind == "lane" else None
                for previous in neighbors:
                    entry = self.contact_centers[previous, rid]
                    a = np.asarray(entry, float)
                    compiled = {}
                    for nxt in sorted(neighbors):
                        b = np.asarray(self.contact_centers[rid, nxt], float)
                        direction = None
                        reason = None
                        if kind == "lane":
                            advance = float((b - a) @ axis)
                            if abs(advance) < .01:
                                reason = "zero_advance_rejected"
                            else:
                                direction_array = axis * (1 if advance > 0 else -1)
                                direction = tuple(map(float, direction_array))
                                if not self._allowed_direction(rid, direction_array, {}):
                                    reason = "direction_rejected"
                        elif kind == "junction_area":
                            sem = lambda region: self.router.features[
                                self.regions[region]["name"]]["semantic_id"]
                            if [sem(previous), sem(rid), sem(nxt)] in self.router.rules.get(
                                    "forbidden_transitions", []):
                                reason = "forbidden_turn_rejected"
                        compiled[nxt] = (entry, tuple(map(float, b)), direction,
                                         float(np.linalg.norm(b - a)), reason)
                    self._static_transitions[previous, rid] = compiled
        self._directed_state_index = {}
        self._directed_distances = None
        self._directed_precompute_ms = 0.0
        if (os.environ.get('THREEYD_V6_PRECOMPUTED_POTENTIAL', '0') == '1'
                and self._static_transitions):
            started = time.perf_counter()
            states = sorted((rid, previous) for previous, rid in
                            self._static_transitions if
                            self.regions[previous]['kind'] != 'parking_area' and
                            self.regions[rid]['kind'] != 'parking_area')
            # A float64 all-pairs matrix is bounded to 16 MiB. Larger maps
            # keep r5's per-query reverse Dijkstra without extra residency.
            if len(states) ** 2 * 8 <= 16 * 2**20:
                index = {state: i for i, state in enumerate(states)}
                row, column, weight = [], [], []
                for (rid, previous), source in index.items():
                    for nxt, prepared in self._static_transitions[previous, rid].items():
                        target = index.get((nxt, rid))
                        if target is None or prepared[4] is not None:
                            continue
                        row.append(source)
                        column.append(target)
                        weight.append(prepared[3])
                graph = csr_matrix((np.asarray(weight, dtype=np.float64),
                                    (row, column)), shape=(len(states), len(states)))
                self._directed_distances = dijkstra(graph, directed=True)
                self._directed_state_index = index
            self._directed_precompute_ms = (time.perf_counter() - started) * 1000

    def _compile_route(self, query, sequence, pieces, ends, cost, expanded):
        sr, sroad, _sb, gr, groad, _gb = ends
        roads, entries = [], []
        for index, (road_id, begin, end, direction) in enumerate(pieces):
            if direction is None:
                continue
            line = right_line(self.router.polygons[road_id], direction, begin, end)
            junction_after = None
            for next_id, _, _, next_direction in pieces[index + 1:]:
                if next_direction is not None:
                    break
                if self.regions[next_id]["kind"] == "junction_area":
                    junction_after = next_id
                    break
            entries.append((road_id, line, junction_after))
            roads.append((road_id, direction))
        points, joins = join_selected_road_lines(entries, self)
        if len(points) < 2:
            return None
        regions = [sr] if sr != sroad else []
        expanded_roads = []
        for road_id, begin, end, direction in pieces:
            regions.append(road_id)
            if direction is None:
                continue
            expanded_roads.append((road_id, direction))
            crossed = []
            for junction in self.adj[road_id]:
                if self.regions[junction]["kind"] != "junction_area" or junction in sequence:
                    continue
                t = float(np.mean(self.contacts[road_id, junction], axis=0) @ direction)
                if begin @ direction + .3 < t < end @ direction - .3:
                    crossed.append((t, junction))
            for _, junction in sorted(crossed):
                regions.extend([junction, road_id])
                expanded_roads.append((road_id, direction))
        if gr != groad:
            regions.append(gr)
        task = {
            "schema": self.SCHEMA,
            "right_reference_polyline": points.tolist(),
            "region_sequence": regions,
            "start_xy": list(query.start[:2]),
            "goal_xy": list(query.goal[:2]),
            "frame_id": "map",
            "map_hash": self._map_hash,
            "rules_hash": self.router.key,
            "junction_joins": joins,
            "selected_route_has_branches": False,
        }
        signature = canonical_hash(task)
        task["signature"] = signature
        self.tasks[signature] = task
        endpoint_access = {}
        for role, region, road in [("start", sr, sroad), ("goal", gr, groad)]:
            binding = next((item for item in self.access.get("bindings", [])
                            if item.get("query_id") == query.query_id
                            and item.get("endpoint") == role), {})
            endpoint_access[role] = {
                "semantic_region_id": region,
                "road_id": road,
                "allow_local_cross_road": bool(binding.get("allow_local_cross_road", False)),
                "parking_binding_source": "confirmed parking_access.json" if binding else "semantic endpoint",
            }
        self.contexts[signature] = {
            "roads": expanded_roads, "unique_roads": roads,
            "endpoint_access": endpoint_access,
        }
        return TopologyRoute(
            tuple(regions), (), tuple(self.parent_semantics[x] for x in regions),
            tuple(query.start), tuple(query.goal), signature,
            {"l1_expanded": expanded, "l1_path_cost_m": cost},
            {"world_key": self.world.key, "topology_key": self.key, **self.world.binding},
        )

    def routes(self, query, limit=1):
        call_started = time.perf_counter()
        # Candidate count is part of the cache identity: an earlier plan()
        # request for one route must not hide later fallback candidates.
        key = (query.query_id, tuple(query.start), tuple(query.goal), limit)
        self._last_first_phase_resume = False
        if key in self._cache:
            self._probe_stats = {"cache_hit": True, "candidate_count": len(self._cache[key][:limit]),
                                 "wall_ms": (time.perf_counter() - call_started) * 1000}
            return self._cache[key][:limit]
        if limit < 1:
            return []
        # The full planner's three-candidate budget predates A*. Preserve its
        # validated candidates after the A* first choice, so a shorter L1
        # route cannot crowd a downstream-feasible route out of that budget.
        legacy_routes = []
        legacy_ms = 0.0
        use_legacy = os.environ.get("THREEYD_L1_LEGACY_COMPAT", "0") == "1"
        # For limit=1 an admissible A* first route always precedes the legacy
        # candidates. Computing legacy alternatives here cannot affect output.
        # Defer that work unless A* fails to produce any route.
        first_budget = int(os.environ.get("THREEYD_L1_FIRST_BUDGET", "200000"))
        alternate_budget = int(os.environ.get("THREEYD_L1_ALTERNATE_BUDGET", "200000"))
        if not 1 <= first_budget <= 1000000 or not 0 <= alternate_budget <= 1000000:
            raise ValueError("L1_SEARCH_BUDGET_OUT_OF_RANGE")

        # Each pair retains its start and goal bindings. The terminal target is
        # the goal road contact for parking, because parking-local travel is not
        # charged by the original L1 cost function.
        pairs = []
        for sr, sroad, sb in self.endpoint_options(query, "start"):
            start = np.asarray(query.start[:2], float)
            if sr != sroad:
                contacts = np.asarray(self.contacts[sr, sroad], float)
                start = contacts[np.argmin(np.linalg.norm(contacts - start, axis=1))]
            for gr, groad, gb in self.endpoint_options(query, "goal"):
                terminal = np.asarray(query.goal[:2], float)
                if gr != groad:
                    contacts = np.asarray(self.contacts[groad, gr], float)
                    terminal = contacts[np.argmin(np.linalg.norm(contacts - terminal, axis=1))]
                    lower = contacts.min(axis=0)
                    upper = contacts.max(axis=0)
                else:
                    lower = upper = terminal
                pairs.append({
                    "ends": (sr, sroad, sb, gr, groad, gb),
                    "start": tuple(map(float, start)),
                    "terminal": tuple(map(float, terminal)),
                    "target_box": (float(lower[0]), float(upper[0]),
                                   float(lower[1]), float(upper[1])),
                })

        # A path that starts and ends in the same lane cannot leave it and
        # return: both enumerators forbid revisiting a region. This exact
        # certificate is deliberately limited to a single non-parking binding.
        single_lane_candidate = False
        if (os.environ.get("THREEYD_V6_SINGLE_LANE_CERTIFICATE", "0") == "1"
                and len(pairs) == 1):
            sr, sroad, _sb, gr, groad, _gb = pairs[0]["ends"]
            single_lane_candidate = (sr == sroad == gr == groad
                                     and self.regions[sroad]["kind"] == "lane")
        if use_legacy and limit > 1 and not single_lane_candidate:
            legacy_started = time.perf_counter()
            legacy_key = (query.query_id, tuple(query.start[:2]), tuple(query.goal[:2]))
            self._cache.pop(legacy_key, None)
            legacy_routes = V5Topology.routes(self, query, limit=limit)
            legacy_ms = (time.perf_counter() - legacy_started) * 1000

        use_goal_potential = os.environ.get("THREEYD_L1_GOAL_POTENTIAL", "0") == "1"
        use_directed_potential = (use_goal_potential and
            os.environ.get("THREEYD_V6_DIRECTED_POTENTIAL", "0") == "1" and
            bool(self._static_transitions))
        skip_relaxed = (use_directed_potential and
            os.environ.get('THREEYD_V6_SKIP_RELAXED_POTENTIAL', '0') == '1')
        potential_started = time.perf_counter()
        resume_key = (query.query_id, tuple(query.start), tuple(query.goal),
                      first_budget, use_goal_potential, use_directed_potential,
                      skip_relaxed, self._directed_distances is not None)
        if use_goal_potential:
            # Exact relaxed-distance lower bound on contact states. The
            # relaxation ignores traffic and visited-region restrictions, so
            # it cannot overestimate a legal route. It is only used for the
            # first candidate; alternate candidate ordering stays unchanged.
            for pair in pairs:
                goal = pair["ends"][4]
                terminal = pair["terminal"]
                if not skip_relaxed:
                    bounds = {}
                    queue = []
                    for previous in self.adj[goal]:
                        entry = self.contact_centers[goal, previous]
                        distance = math.hypot(entry[0] - terminal[0], entry[1] - terminal[1])
                        bounds[(goal, previous)] = distance
                        heapq.heappush(queue, (distance, goal, previous))
                    while queue:
                        distance, region, previous = heapq.heappop(queue)
                        if distance > bounds[(region, previous)]:
                            continue
                        exit_xy = self.contact_centers[previous, region]
                        for before in self.adj[previous]:
                            entry = self.contact_centers[previous, before]
                            step = math.hypot(entry[0] - exit_xy[0], entry[1] - exit_xy[1])
                            key2 = (previous, before)
                            proposed = distance + step
                            if proposed < bounds.get(key2, math.inf):
                                bounds[key2] = proposed
                                heapq.heappush(queue, (proposed, previous, before))
                    pair["relaxed_bounds"] = bounds
                    start_road = pair["ends"][1]
                    start_xy = pair["start"]
                    start_bound = (math.hypot(start_xy[0] - terminal[0], start_xy[1] - terminal[1])
                                   if start_road == goal else math.inf)
                    for next_region in self.adj[start_road]:
                        exit_xy = self.contact_centers[start_road, next_region]
                        tail = bounds.get((next_region, start_road))
                        if tail is not None:
                            start_bound = min(start_bound, math.hypot(
                                start_xy[0] - exit_xy[0], start_xy[1] - exit_xy[1]) + tail)
                    pair["start_bound"] = start_bound
                else:
                    # The straight-line bound is always admissible when the
                    # directed state does not match a compiled contact.
                    pair['relaxed_bounds'] = {}
                    pair['start_bound'] = 0.0
                    start_road = pair['ends'][1]
                    start_xy = pair['start']
                if use_directed_potential:
                    # Reverse Dijkstra on (previous,current) contact states.
                    # It honors static travel direction and forbidden turns,
                    # but permits revisits. This relaxation is a lower bound
                    # on the simple-region path searched by the forward A*.
                    directed = {}
                    directed_queue = []
                    gb = pair["ends"][5]
                    for previous in self.adj[goal]:
                        if self.regions[previous]["kind"] == "parking_area":
                            continue
                        entry = np.asarray(self.contact_centers[previous, goal], float)
                        delta = np.asarray(terminal, float) - entry
                        if self.regions[goal]["kind"] == "lane":
                            axis = self.router.axes[goal]
                            advance = float(delta @ axis)
                            if abs(advance) < .01:
                                continue
                            direction = axis * (1 if advance > 0 else -1)
                            if not self._allowed_direction(goal, direction, {}):
                                continue
                            if not self._allowed_direction(goal, direction, gb):
                                continue
                        cost = float(np.linalg.norm(delta))
                        directed[goal, previous] = cost
                        heapq.heappush(directed_queue, (cost, goal, previous))
                    if self._directed_distances is None:
                        while directed_queue:
                            cost, region, previous = heapq.heappop(directed_queue)
                            if cost > directed[region, previous]:
                                continue
                            if self.regions[previous]["kind"] == "parking_area":
                                continue
                            for before in self.adj[previous]:
                                if self.regions[before]["kind"] == "parking_area":
                                    continue
                                prepared = self._static_transitions.get(
                                    (before, previous), {}).get(region)
                                if prepared is None or prepared[4] is not None:
                                    continue
                                state = (previous, before)
                                proposed = cost + prepared[3]
                                if proposed < directed.get(state, math.inf):
                                    directed[state] = proposed
                                    heapq.heappush(directed_queue,
                                                   (proposed, previous, before))
                    else:
                        seed_indices = [self._directed_state_index[state]
                                        for state in directed]
                        if seed_indices:
                            terminal_costs = np.asarray(list(directed.values()))
                            bound_vector = np.min(
                                self._directed_distances[:, seed_indices]
                                + terminal_costs[None, :], axis=1)
                            directed = {state: float(bound_vector[index])
                                        for state, index in self._directed_state_index.items()
                                        if math.isfinite(bound_vector[index])}
                        else:
                            directed = {}
                    pair["directed_bounds"] = directed
                    start = np.asarray(start_xy, float)
                    directed_start = math.inf
                    if start_road == goal:
                        delta = np.asarray(terminal, float) - start
                        valid = True
                        if self.regions[start_road]["kind"] == "lane":
                            axis = self.router.axes[start_road]
                            advance = float(delta @ axis)
                            valid = abs(advance) >= .01
                            if valid:
                                direction = axis * (1 if advance > 0 else -1)
                                valid = (self._allowed_direction(start_road, direction,
                                         pair["ends"][2]) and
                                         self._allowed_direction(start_road, direction, gb))
                        if valid:
                            directed_start = float(np.linalg.norm(delta))
                    for nxt in self.adj[start_road]:
                        if self.regions[nxt]["kind"] == "parking_area":
                            continue
                        tail = directed.get((nxt, start_road))
                        if tail is None:
                            continue
                        delta = np.asarray(self.contact_centers[start_road, nxt], float) - start
                        if self.regions[start_road]["kind"] == "lane":
                            axis = self.router.axes[start_road]
                            advance = float(delta @ axis)
                            if abs(advance) < .01:
                                continue
                            direction = axis * (1 if advance > 0 else -1)
                            if not self._allowed_direction(start_road, direction,
                                                           pair["ends"][2]):
                                continue
                        directed_start = min(directed_start,
                                             float(np.linalg.norm(delta)) + tail)
                    pair["directed_start_bound"] = directed_start
        goal_potential_ms = (time.perf_counter() - potential_started) * 1000

        def heuristic(entry, pair, region=None, previous=None, relaxed=False):
            xmin, xmax, ymin, ymax = pairs[pair]["target_box"]
            x, y = entry
            euclidean = math.hypot(max(xmin - x, 0., x - xmax),
                                   max(ymin - y, 0., y - ymax))
            if not relaxed:
                return euclidean
            bound = (pairs[pair]["start_bound"] if previous is None else
                     pairs[pair]["relaxed_bounds"].get((region, previous), euclidean))
            if use_directed_potential:
                if previous is None:
                    bound = pairs[pair]["directed_start_bound"]
                elif entry == self.contact_centers.get((previous, region)):
                    bound = pairs[pair]["directed_bounds"].get(
                        (region, previous), math.inf)
            return max(euclidean, bound - 1e-7) if math.isfinite(bound) else euclidean

        def unpack(nodes, terminal_id):
            sequence, pieces = [], []
            node_id = terminal_id
            while node_id >= 0:
                node = nodes[node_id]
                if node.rid is not None:
                    sequence.append(node.rid)
                if node.piece is not None:
                    road_id, begin, end, direction = node.piece
                    pieces.append((road_id, np.asarray(begin, float),
                                   np.asarray(end, float),
                                   None if direction is None else np.asarray(direction, float)))
                node_id = node.parent
            return tuple(reversed(sequence)), list(reversed(pieces))

        found, seen = [], set()
        metrics = {}
        cumulative_expansions = 0

        def search(dominance, budget, target_count, phase):
            nonlocal cumulative_expansions
            phase_started = time.perf_counter()
            nodes, heap = [], []
            serial = itertools.count()
            labels = defaultdict(list)
            use_count_buckets = (dominance and os.environ.get('THREEYD_V6_DOMINANCE_BUCKETS', '0') == '1')
            copy_on_delete = os.environ.get('THREEYD_V6_COPY_ON_DELETE', '0') == '1'
            labels_by_count = defaultdict(lambda: defaultdict(list)) if use_count_buckets else None
            inactive = set()
            counters = defaultdict(int)
            peak = 0

            def push(node):
                if dominance and node.rid is not None:
                    state = (node.pair, node.previous, node.rid, node.entry)
                    visited_count = node.visited.bit_count()
                    if use_count_buckets:
                        by_count = labels_by_count[state]
                        for count, bucket in by_count.items():
                            if count > visited_count:
                                continue
                            for old_mask, old_cost, _ in bucket:
                                if old_cost <= node.cost + 1e-9 and old_mask & ~node.visited == 0:
                                    counters['dominated_prefixes'] += 1
                                    return
                        for count, bucket in by_count.items():
                            if count < visited_count:
                                continue
                            if copy_on_delete:
                                keep = None
                                for index, (old_mask, old_cost, old_id) in enumerate(bucket):
                                    if node.cost <= old_cost + 1e-9 and node.visited & ~old_mask == 0:
                                        if keep is None:
                                            keep = bucket[:index]
                                        inactive.add(old_id)
                                    elif keep is not None:
                                        keep.append((old_mask, old_cost, old_id))
                                if keep is not None:
                                    by_count[count] = keep
                            else:
                                keep = []
                                for old_mask, old_cost, old_id in bucket:
                                    if node.cost <= old_cost + 1e-9 and node.visited & ~old_mask == 0:
                                        inactive.add(old_id)
                                    else:
                                        keep.append((old_mask, old_cost, old_id))
                                by_count[count] = keep
                        by_count[visited_count].append((node.visited, node.cost, len(nodes)))
                    else:
                        old_labels = labels[state]
                        # A mask with more set bits cannot be a subset.
                        for old_mask, old_cost, _, old_count in old_labels:
                            if (old_count <= visited_count
                                    and old_cost <= node.cost + 1e-9
                                    and old_mask & ~node.visited == 0):
                                counters["dominated_prefixes"] += 1
                                return
                        keep = []
                        for old_mask, old_cost, old_id, old_count in old_labels:
                            if (visited_count <= old_count
                                    and node.cost <= old_cost + 1e-9
                                    and node.visited & ~old_mask == 0):
                                inactive.add(old_id)
                            else:
                                keep.append((old_mask, old_cost, old_id, old_count))
                        keep.append((node.visited, node.cost, len(nodes), visited_count))
                        labels[state] = keep
                node_id = len(nodes)
                nodes.append(node)
                f = (node.cost if node.rid is None else node.cost + heuristic(
                    node.entry, node.pair, node.rid, node.previous,
                    relaxed=use_goal_potential and phase == "first"))
                heapq.heappush(heap, (f, node.cost, next(serial), node_id))
                counters["pushed"] += 1

            for pair_id, pair in enumerate(pairs):
                road = pair["ends"][1]
                push(_Node(road, None, pair["start"], 1 << road,
                           pair_id, -1, None, 0.))
            expanded = 0
            while heap and expanded < budget and len(found) < target_count:
                _, _, _, node_id = heapq.heappop(heap)
                if node_id in inactive:
                    counters["inactive_skipped"] += 1
                    continue
                node = nodes[node_id]
                expanded += 1
                cumulative_expansions += 1
                if node.rid is None:
                    sequence, pieces = unpack(nodes, node_id)
                    if sequence in seen:
                        counters["duplicate_complete_routes"] += 1
                        continue
                    seen.add(sequence)
                    try:
                        route = self._compile_route(
                            query, sequence, pieces, pairs[node.pair]["ends"],
                            node.cost, cumulative_expansions)
                    except ValueError:
                        counters["geometry_rejected"] += 1
                        continue
                    if route is None:
                        counters["geometry_rejected"] += 1
                        continue
                    found.append(route)
                    counters["completed_candidates"] += 1
                    continue

                rid = node.rid
                sr, sroad, sb, gr, groad, gb = pairs[node.pair]["ends"]
                a = None
                compiled = self._static_transitions.get((node.previous, rid))
                choices = ([None] if rid == groad else []) + sorted(self.adj[rid])
                for nxt in choices:
                    terminal = nxt is None
                    if not terminal:
                        if node.visited & (1 << nxt) or self.regions[nxt]["kind"] == "parking_area":
                            counters["cycle_or_parking_rejected"] += 1
                            continue
                    prepared = compiled.get(nxt) if compiled is not None and not terminal else None
                    if prepared is not None and node.entry == prepared[0]:
                        _, b_tuple, direction_tuple, step, reason = prepared
                        if reason is not None:
                            counters[reason] += 1
                            continue
                        piece = (rid, node.entry, b_tuple, direction_tuple)
                        push(_Node(nxt, rid, b_tuple,
                                   node.visited | (1 << nxt), node.pair, node_id,
                                   piece, node.cost + step))
                        continue
                    if a is None:
                        a = np.asarray(node.entry, float)
                    b = (np.asarray(pairs[node.pair]["terminal"], float) if terminal
                         else np.asarray(self.contact_centers[rid, nxt], float))
                    direction = None
                    if self.regions[rid]["kind"] == "lane":
                        axis = self.router.axes[rid]
                        advance = float((b - a) @ axis)
                        if abs(advance) < .01:
                            counters["zero_advance_rejected"] += 1
                            continue
                        direction = axis * (1 if advance > 0 else -1)
                        if not self._allowed_direction(
                                rid, direction, sb if rid == sroad and node.parent == -1 else {}):
                            counters["direction_rejected"] += 1
                            continue
                        if terminal and not self._allowed_direction(rid, direction, gb):
                            counters["goal_direction_rejected"] += 1
                            continue
                    if node.previous is not None and self.regions[rid]["kind"] == "junction_area" and not terminal:
                        sem = lambda region: self.router.features[self.regions[region]["name"]]["semantic_id"]
                        if [sem(node.previous), sem(rid), sem(nxt)] in self.router.rules.get("forbidden_transitions", []):
                            counters["forbidden_turn_rejected"] += 1
                            continue
                    piece = (rid, node.entry, tuple(map(float, b)),
                             None if direction is None else tuple(map(float, direction)))
                    new_cost = node.cost + float(np.linalg.norm(b - a))
                    push(_Node(None if terminal else nxt, rid,
                               tuple(map(float, b)),
                               node.visited if terminal else node.visited | (1 << nxt),
                               node.pair, node_id, piece, new_cost))
                peak = max(peak, len(heap))
            metrics[phase] = {
                "expanded": expanded,
                "budget": budget,
                "budget_reached": bool(heap) and expanded >= budget,
                "queue_remaining": len(heap),
                "queue_peak": peak,
                "allocated_nodes": len(nodes),
                "wall_ms": (time.perf_counter() - phase_started) * 1000,
                **counters,
            }

        resume = None
        if (limit > 1 and os.environ.get("THREEYD_V6_LAZY_RESUME", "0") == "1"):
            resume = self._first_phase_cache.pop(resume_key, None)
        if resume is None:
            search(True, first_budget, 1, "first")
            if limit == 1 and os.environ.get("THREEYD_V6_LAZY_RESUME", "0") == "1":
                self._first_phase_cache[resume_key] = (list(found), set(seen),
                                                       cumulative_expansions, dict(metrics))
                while len(self._first_phase_cache) > 16:
                    self._first_phase_cache.popitem(last=False)
        else:
            found, seen, cumulative_expansions, metrics = resume
            self._last_first_phase_resume = True
        # The certificate limits candidate cardinality; it does not certify
        # that the budgeted A* found that candidate. Preserve the legacy
        # fallback if the first pass exhausted its budget without a route.
        if single_lane_candidate and not found and use_legacy and limit > 1:
            legacy_started = time.perf_counter()
            legacy_key = (query.query_id, tuple(query.start[:2]), tuple(query.goal[:2]))
            self._cache.pop(legacy_key, None)
            legacy_routes = V5Topology.routes(self, query, limit=limit)
            legacy_ms = (time.perf_counter() - legacy_started) * 1000
        # Final ordering is first A*, then legacy candidates, then remaining
        # A*. If the first two groups already contain `limit` distinct route
        # signatures, no alternate A* result can enter the public output.
        prefix_complete = False
        if (os.environ.get("THREEYD_V6_PREFIX_SUFFICIENCY", "0") == "1"
                and found and legacy_routes and limit > 1):
            prefix_complete = len({route.signature for route in
                                   found[:1] + legacy_routes}) >= limit
        if (found and limit > 1 and alternate_budget and
                not single_lane_candidate and not prefix_complete):
            search(False, alternate_budget, limit, "alternates")
        astar_candidates = len(found)
        if use_legacy and limit == 1 and not found:
            legacy_key = (query.query_id, tuple(query.start[:2]), tuple(query.goal[:2]))
            self._cache.pop(legacy_key, None)
            legacy_routes = V5Topology.routes(self, query, limit=limit)
        if legacy_routes:
            ordered = found[:1] + legacy_routes + found[1:]
            unique = []
            signatures = set()
            for route in ordered:
                if route.signature not in signatures:
                    signatures.add(route.signature)
                    unique.append(route)
                if len(unique) >= limit:
                    break
            found = unique
        self._probe_stats = {
            "algorithm": "A* admissible goal-contact lower bound",
            "first_route_dominance": True,
            "alternatives_without_dominance": True,
            "total_expansions": cumulative_expansions,
            "candidate_count": len(found),
            "astar_candidate_count": astar_candidates,
            "legacy_compat_candidate_count": len(legacy_routes),
            "first_route_found": bool(astar_candidates),
            "phases": metrics,
            "first_phase_resumed": self._last_first_phase_resume,
            "single_lane_candidate_certificate": single_lane_candidate,
            "alternate_prefix_complete": prefix_complete,
            "legacy_ms": legacy_ms,
            "goal_potential_ms": goal_potential_ms,
            "directed_goal_potential": use_directed_potential,
            "precomputed_goal_potential": self._directed_distances is not None,
            "skip_relaxed_potential": skip_relaxed,
            "wall_ms": (time.perf_counter() - call_started) * 1000,
        }
        self._cache[key] = found
        return found[:limit]
