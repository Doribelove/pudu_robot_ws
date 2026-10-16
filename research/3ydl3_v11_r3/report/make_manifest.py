"""Register the final measured code, raw observations and delivered report."""
from pathlib import Path
import json,hashlib
R=Path(__file__).resolve().parents[1]
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def hashes(files):return {str(p.relative_to(R)):sha(p) for p in sorted(files) if p.is_file() and '__pycache__' not in str(p)}
assert read(R/'ACCEPTANCE.json')['full_performance_acceptance']
assert read(R/'report/FINAL_PROTECTION.json')['all_owned_cleanup_complete']
lock=read(R/'results/paired_final/source.json');assert all(sha(Path(p))==h for p,h in lock.items())
batches=list((R/'results/paired_final').glob('round*/*/*'))
casefiles=[p for base in batches if base.is_dir() for p in (base/'cases').rglob('*') if p.is_file()]
records=[p for base in batches if base.is_dir() for p in base.iterdir() if p.is_file()]
record={'version':'3YDL3-V11-r3','workspace':str(R),'formal_results':'results/paired_final','measured_core_file_count':len(lock),'measured_core_hashes':lock,'test_hashes':hashes((R/'tests').rglob('*')),'test_auxiliary_hashes':hashes((R/'tests/oracles').rglob('*')),'protection_lock_hashes':hashes([R/'PARENT_PROTECTION.json',*R.glob('INTERMEDIATE*PROTECTION.json')]),'formal_snapshot_hashes':hashes(p for base in batches if base.is_dir() for p in (base/'source_snapshot').rglob('*')),'formal_batch_record_hashes':hashes(records),'formal_case_output_hashes':hashes(casefiles),'owned_process_journal_hashes':hashes((R/'results').rglob('*.supervisor.json')),'report_hashes':hashes(p for p in (R/'report').iterdir() if p.is_file()),'entry_documents':hashes(R/name for name in ['README.md','ACCEPTANCE.json','TASK.json','BASELINE.json','MAPS.json','L1_LOCK.json']),'exclusions':'All A–J previous candidates, current pilot, profiles, event/order/default diagnostics are retained and excluded from formal performance statistics.'}
(R/'DELIVERY_MANIFEST.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
for group in ['test_hashes','test_auxiliary_hashes','protection_lock_hashes','formal_snapshot_hashes','formal_batch_record_hashes','formal_case_output_hashes','owned_process_journal_hashes','report_hashes','entry_documents']:
 assert all(sha(R/p)==h for p,h in record[group].items()),group
print(json.dumps({'measured_core_files':len(lock),'case_files':len(record['formal_case_output_hashes']),'reports':len(record['report_hashes']),'manifest_verified':True}))
