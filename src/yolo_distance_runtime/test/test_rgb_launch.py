import ast
import json
from pathlib import Path
import tempfile
import unittest


class Config:
    def __init__(self,name):self.name=name
    def perform(self,context):return context[self.name]


class RgbLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root=Path(__file__).parents[1]
        path=cls.root/'launch/rgb_distance.launch.py'
        function=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='_start')
        namespace=dict(Path=Path,json=json,get_package_share_directory=lambda _:str(cls.root),
                       LaunchConfiguration=Config,ParameterValue=lambda c,**kw:c,Node=lambda **kw:kw)
        exec(compile(ast.Module(body=[function],type_ignores=[]),str(path),'exec'),namespace)
        cls.start=staticmethod(namespace['_start'])

    def test_rgb_only_model_selection_has_one_camera_node(self):
        for name in ['mlp','lightgbm']:
            actions=self.start(dict(model=name,distance_model='',compare_stereo='false'))
            self.assertEqual(len(actions),1)
            node=actions[0];self.assertEqual(node['package'],'yolo_distance_runtime')
            settings=node['parameters'][1]
            self.assertEqual(settings['mode'],'rgb_infer')
            self.assertEqual(Path(settings['distance_model']).name,name+'_rgb.json')
            self.assertNotIn('stereo',settings)

    def test_stereo_comparison_keeps_portable_model_and_selects_reference_mode(self):
        for name in ['mlp','lightgbm']:
            actions=self.start(dict(model=name,distance_model='',compare_stereo='true'))
            self.assertEqual(actions[0]['parameters'][1]['mode'],'rgb_compare')
            self.assertEqual(Path(actions[0]['parameters'][1]['distance_model']).name,name+'_rgb.json')
        with self.assertRaisesRegex(RuntimeError,'compare_stereo'):
            self.start(dict(model='mlp',distance_model='',compare_stereo='maybe'))

    def test_missing_model_and_wrong_algorithm_fail_before_camera_start(self):
        with self.assertRaisesRegex(RuntimeError,'must be'):
            self.start(dict(model='unknown',distance_model=''))
        with self.assertRaisesRegex(RuntimeError,'missing'):
            self.start(dict(model='mlp',distance_model='/tmp/absent_rgb_distance_model_9274.json'))
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'wrong.json';p.write_text(json.dumps(dict(version=3,algorithm='mlp')))
            with self.assertRaisesRegex(RuntimeError,'match'):
                self.start(dict(model='lightgbm',distance_model=str(p)))

if __name__=='__main__':unittest.main()
