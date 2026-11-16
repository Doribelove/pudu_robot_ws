import ast,copy,hashlib,json,importlib.machinery
EXPECTED='27bc6c2b90d6b2301b05331e8854d27ce6cbf6f24f5d06a005321025a234c001'
RHS="Path('/isolated_bundle/runtime/resources/semantic')"
def ast_identity(tree):return hashlib.sha256(ast.dump(tree,include_attributes=False).encode()).hexdigest()
def validate_single_change(original,derived):
 before=copy.deepcopy(original);matches=[n for n in before.body if isinstance(n,ast.Assign) and len(n.targets)==1 and isinstance(n.targets[0],ast.Name) and n.targets[0].id=='WORK']
 if len(matches)!=1 or ast.unparse(matches[0].value)!="Path(__file__).resolve().parents[6]":raise ValueError('EXACT_WORK_ASSIGNMENT_REQUIRED')
 matches[0].value=ast.parse(RHS,mode='eval').body
 if ast.dump(before,include_attributes=False)!=ast.dump(derived,include_attributes=False):raise ValueError('MULTIPLE_OR_WRONG_AST_CHANGE')
def adapt_code(source,filename):
 if hashlib.sha256(source).hexdigest()!=EXPECTED:raise ValueError('ORIGINAL_SOURCE_SHA_MISMATCH')
 original=ast.parse(source,filename=filename);derived=copy.deepcopy(original)
 matches=[n for n in derived.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='WORK' for t in n.targets)]
 if len(matches)!=1:raise ValueError('WORK_ASSIGNMENT_COUNT')
 matches[0].value=ast.parse(RHS,mode='eval').body;validate_single_change(original,derived)
 identity={'source_sha256':EXPECTED,'original_ast_sha256':ast_identity(original),'derived_ast_sha256':ast_identity(derived),'filename_unchanged':filename,'adaptation':'one WORK RHS only; derived code not original-byte execution'}
 return compile(ast.fix_missing_locations(derived),filename,'exec'),identity
class ExactSemanticLoader(importlib.machinery.SourceFileLoader):
 def __init__(self,name,path,records,identities):super().__init__(name,path);self.records=records;self.identities=identities
 def get_code(self,fullname):
  source=self.get_data(self.path);code,self.derived_identity=adapt_code(source,self.path)
  self.execution_identity={'module':fullname,'loader':'ExactSemanticLoader','derived_identity':self.derived_identity,'execution_status':'compiled_not_yet_executed'}
  if len(self.identities)>=4:raise ValueError('DERIVED_IDENTITY_EVENT_LIMIT')
  self.identities.append(self.execution_identity);self.records.append(dict(self.execution_identity));return code
 def exec_module(self,module):
  try:super().exec_module(module)
  except BaseException as e:
   if hasattr(self,'execution_identity'):self.execution_identity.update(execution_status='module_execution_failed',exception_type=type(e).__name__)
   raise
  else:self.execution_identity['execution_status']='module_execution_completed'
def diagnostic_admission(ad):
 required={'execution_authorized','mode','ast_adapter_enabled'}
 if not required.issubset(ad) or ad['execution_authorized'] is not True or ad['mode']!='import-diagnostic' or type(ad['ast_adapter_enabled']) is not bool:raise ValueError('DIAGNOSTIC_ADMISSION_FAIL_CLOSED')
 return ad['ast_adapter_enabled']
def bounded_trace(exc,source_hashes,loader_records):
 frames=[];tb=exc.__traceback__;total=0
 while tb is not None:
  if len(frames)<16:
   f=tb.tb_frame;path=f.f_code.co_filename;frames.append({'file':path[:500],'line':tb.tb_lineno,'function':f.f_code.co_name[:120],'source_sha256':source_hashes.get(path),'source_locked':path in source_hashes})
  total+=1;tb=tb.tb_next
 report={'type':type(exc).__name__,'message':str(exc)[:1000],'frames':frames,'frames_truncated':total>16,'loaders':loader_records[-32:]}
 b=json.dumps(report,ensure_ascii=True).encode()
 if len(b)>65536:return {'type':type(exc).__name__,'trace_output_limit':True,'truncated':True,'frames':frames}
 return report
