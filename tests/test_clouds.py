import unittest
from types import SimpleNamespace as Field
import numpy as np
from semantic_search.clouds import sample_cloud


class CloudTests(unittest.TestCase):
    def test_padded_rows_and_big_endian_preserve_records(self):
        fields=[Field(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')]
        for endian in ['<','>']:
            data=bytearray(2*64)
            a=np.ndarray((2,4,3),dtype=endian+'f4',buffer=data,strides=(64,12,4))
            a[:]=np.arange(24).reshape(2,4,3);a[1,2,0]=np.nan
            output=sample_cloud(data,4,2,12,64,fields,endian=='>',1)
            points=np.frombuffer(output,dtype=endian+'f4').reshape(-1,3)
            self.assertEqual(len(points),7)
            np.testing.assert_array_equal(points[4],a[1,0])
            down=sample_cloud(data,4,2,12,64,fields,endian=='>',2)
            self.assertEqual(len(down),2*12)

    def test_truncated_and_wrong_typed_fields_rejected(self):
        fields=[Field(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')]
        with self.assertRaises(ValueError):sample_cloud(b'\0'*10,1,1,12,12,fields)
        fields[0].datatype=8
        with self.assertRaises(ValueError):sample_cloud(b'\0'*12,1,1,12,12,fields)
