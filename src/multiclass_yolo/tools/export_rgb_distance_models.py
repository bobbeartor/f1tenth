#!/usr/bin/env python3
"""Export fitted benchmark weights into a self-contained C++ CPU inference format."""
import argparse
import hashlib
import json
from pathlib import Path


def prune(tree):
    if 'leaf_value' in tree:
        return dict(leaf=tree['leaf_value'])
    result=dict(feature=tree['split_feature'],default_left=tree['default_left'],missing_type=tree['missing_type'])
    if tree['decision_type']=='==':
        result['categories']=[int(c) for c in str(tree['threshold']).split('||')]
    elif tree['decision_type']=='<=':
        result['threshold']=float(tree['threshold'])
    else:
        raise ValueError('Unsupported LightGBM split operator')
    result['left'],result['right']=prune(tree['left_child']),prune(tree['right_child'])
    return result


def export(run, output):
    import lightgbm as lgb
    run,output=Path(run),Path(output)
    metadata=json.loads((run/'metadata.json').read_text())
    if metadata['version']!=2:
        raise ValueError('Expected fitted benchmark_v2 models')
    for name,key in [('cleaned.jsonl','cleaned_sha256'),('splits.json','splits_sha256'),('config.json','config_sha256')]:
        if hashlib.sha256((run/name).read_bytes()).hexdigest()!=metadata[key]:
            raise ValueError('Training provenance changed')
    output.mkdir(parents=True,exist_ok=True)
    common=dict(version=3,feature_names=metadata['feature_names'],distance_definition='rgb_optical_z_m',
                camera_contract=json.loads(metadata['camera_contract']),classes=metadata['train_classes'],
                prediction_range_m=metadata['prediction_range_m'],training_seed=metadata['seed'])
    weights=json.loads((run/'mlp_weights.json').read_text())
    mlp=dict(common,algorithm='mlp',mean=weights['metadata']['mean'],scale=weights['metadata']['scale'],layers=weights['layers'],
             source_model_sha256=hashlib.sha256((run/'mlp_weights.json').read_bytes()).hexdigest())
    ensemble=lgb.Booster(model_file=str(run/'lightgbm.txt')).dump_model()
    if (ensemble['objective'] not in ['huber','regression','regression_l1'] or ensemble['average_output'] or
            ensemble['num_class']!=1 or ensemble['num_tree_per_iteration']!=1 or ensemble['feature_names']!=metadata['feature_names']):
        raise ValueError('Only single-output identity-link GBDT regression is supported')
    lightgbm=dict(common,algorithm='lightgbm',trees=[prune(t['tree_structure']) for t in ensemble['tree_info']],
                  source_model_sha256=hashlib.sha256((run/'lightgbm.txt').read_bytes()).hexdigest())
    report={}
    for name,model in [('mlp',mlp),('lightgbm',lightgbm)]:
        path=output/(name+'_rgb.json');path.write_text(json.dumps(model,separators=(',',':'),allow_nan=False))
        report[name]=dict(file=path.name,bytes=path.stat().st_size,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),source_model_sha256=model['source_model_sha256'])
    (output/'export_manifest.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser(__doc__);parser.add_argument('--run',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();export(args.run,args.output)
