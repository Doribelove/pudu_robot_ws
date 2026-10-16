"""Order existing legal routes by lane-endpoint heading compatibility.

This is a heuristic, never an infeasibility certificate. Every original route
is retained, and only the original L1 generator supplies route candidates.
"""
import math


def endpoint_conflicts(topology,route,query):
    context=topology.contexts[route.signature]
    result=[]
    for role,pose in (('start',query.start),('goal',query.goal)):
        endpoint=context['endpoint_access'][role]
        region=topology.regions[endpoint['semantic_region_id']]
        if region['kind']!='lane':continue
        directions=[d for rid,d in context['roads'] if rid==endpoint['road_id']]
        if not directions:continue
        direction=directions[0 if role=='start' else -1]
        dot=float(direction[0]*math.cos(pose[2])+direction[1]*math.sin(pose[2]))
        if dot<-.5:result.append({'endpoint':role,'road_id':endpoint['road_id'],'alignment':dot})
    return result


def ordered(topology,routes,query):
    scores={r.signature:endpoint_conflicts(topology,r,query) for r in routes}
    return sorted(routes,key=lambda r:len(scores[r.signature])),scores
