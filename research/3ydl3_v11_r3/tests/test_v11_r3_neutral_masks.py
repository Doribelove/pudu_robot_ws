import numpy as np
import pytest
from three_yd_v5.neutral_masks import physical_masks
from three_yd_v5.v9_maps import immutable

@pytest.mark.parametrize('seed',range(8))
@pytest.mark.parametrize('kind',['neutral','modified','reverse'])
def test_each_cell_and_ownership_match_original_even_if_adapter_field_changed(seed,kind):
 rng=np.random.default_rng(seed);hard=rng.random((51,79))>.7;labels=(~hard).astype(np.int16)
 if kind=='modified':labels[5:12,9:21]=1;labels[14:20,24:31]=2
 if kind=='reverse':hard=hard[::-1,::-1];labels=labels[::-1,::-1]
 expected_drivable=labels==1;expected_physical=~hard&expected_drivable
 a,b=physical_masks(hard,labels,rows=7)
 assert np.array_equal(a,expected_drivable) and np.array_equal(b,expected_physical)
 if np.array_equal(expected_drivable,expected_physical):assert a is b and immutable(a)
 else:assert a is not b
 labels[:]=2;hard[:]=True
 assert np.array_equal(a,expected_drivable) and np.array_equal(b,expected_physical)

def test_disabled_storage_preserves_original_separate_mutable_drivable(monkeypatch):
 monkeypatch.setenv('THREEYD_V10_STORAGE','0')
 hard=np.zeros((3,4),bool);a,b=physical_masks(hard,np.ones((3,4),np.int16))
 assert a is not b and a.flags.writeable and np.array_equal(a,b)
