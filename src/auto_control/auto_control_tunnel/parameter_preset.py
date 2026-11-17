"""ROS parameter snapshots, independent of ROS and Tk for offline verification."""
import os
from pathlib import Path
import tempfile

import yaml


def parameter_values(names, values):
    if len(names) != len(values):
        raise ValueError('파라미터 응답 개수가 일치하지 않습니다.')
    fields = {
        1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value',
        5: 'byte_array_value', 6: 'bool_array_value', 7: 'integer_array_value',
        8: 'double_array_value', 9: 'string_array_value',
    }
    result = {}
    for name, value in zip(names, values):
        if value.type == 0:
            raise ValueError(f'값이 없는 파라미터입니다: {name}')
        field = fields.get(value.type)
        if field is None:
            raise ValueError(f'지원하지 않는 파라미터 형식: {name}')
        item = getattr(value, field)
        if value.type >= 5:
            item = list(item)
        if value.type == 5:
            item = [int.from_bytes(v, 'little') if isinstance(v, bytes) else int(v)
                    for v in item]
        result[name] = item
    return result


def write_preset(path, snapshot):
    """Replace the destination only after the complete YAML is written."""
    if not snapshot:
        raise ValueError('저장할 노드가 없습니다.')
    document = {node.lstrip('/'): {'ros__parameters': values}
                for node, values in sorted(snapshot.items())}
    text = '# auto_control runtime preset: applied values only\n' + yaml.safe_dump(
        document, allow_unicode=True, sort_keys=True)
    path = Path(path)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix='.' + path.name, delete=False) as stream:
            temporary = stream.name
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)
