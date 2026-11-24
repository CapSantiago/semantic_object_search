import unittest
from types import SimpleNamespace as Field
import numpy as np
from semantic_search.clouds import sample_cloud,clearing_cloud


class CloudTests(unittest.TestCase):
    def clearing_fixture(self):
        k=[20.,0.,2.,0.,20.,2.,0.,0.,1.];y,x=np.indices((4,4));d=np.ones((4,4))
        points=np.stack([d,-(x+.5-2)/20*d,-(y+.5-2)/20*d],axis=-1).astype('<f4')
        fields=[Field(name=n,offset=i*4,datatype=7,count=1)for i,n in enumerate('xyz')]
        return points,fields,k

    def test_only_verified_positive_infinite_range_becomes_clearing_ray(self):
        points,fields,k=self.clearing_fixture();points[0,0]=[np.inf,np.nan,np.nan];points[0,1]=np.nan;points[0,2]=0;points[0,3]=[-np.inf,0,0]
        output,count=clearing_cloud(points.tobytes(),4,4,12,48,fields,False,k,8.,1)
        values=np.frombuffer(output,dtype='<f4').reshape(-1,3)
        self.assertEqual(count,1);self.assertEqual(len(values),13)
        np.testing.assert_allclose(values[-1],[8.,.6,.6])
        np.testing.assert_array_equal(values[:-1],points.reshape(-1,3)[4:])
        marked=sample_cloud(points.tobytes(),4,4,12,48,fields,False,1,True)
        self.assertFalse(np.any(np.frombuffer(marked,dtype='<f4').reshape(-1,3)[:,0]==8.))

    def test_wrong_camera_geometry_cannot_generate_free_rays(self):
        points,fields,k=self.clearing_fixture();k[2]+=2
        with self.assertRaises(ValueError):clearing_cloud(points.tobytes(),4,4,12,48,fields,False,k,8.,1)

    def test_unverified_all_infinite_image_emits_no_free_space(self):
        points,fields,k=self.clearing_fixture();points[:]=np.inf
        output,count=clearing_cloud(points.tobytes(),4,4,12,48,fields,False,k,8.,1)
        self.assertEqual((output,count),(b'',0))

    def test_compact_xyz_preserves_geometry_with_padding_extra_fields_and_endianness(self):
        fields=[Field(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')]
        for endian in ('<','>'):
            data=bytearray(2*80)
            values=np.ndarray((2,4,3),dtype=endian+'f4',buffer=data,strides=(80,16,4))
            values[:]=np.arange(24).reshape(2,4,3);values[1,2,0]=np.nan
            output=sample_cloud(data,4,2,16,80,fields,endian=='>',1,compact=True)
            points=np.frombuffer(output,dtype='<f4').reshape(-1,3)
            np.testing.assert_array_equal(points,values.reshape(-1,3)[np.isfinite(values).all(axis=2).ravel()])
            self.assertEqual(len(output),7*12)

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
