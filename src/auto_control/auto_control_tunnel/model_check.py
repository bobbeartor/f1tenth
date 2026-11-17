"""Startup model-report bookkeeping; used by the automatic-drive readiness gate."""


class ModelChecks:
    def __init__(self, expected):
        if not expected or len(set(expected)) != len(expected):
            raise ValueError('Expected model names must be nonempty and unique')
        self.status = dict.fromkeys(expected, 'WAITING')

    def update(self, name, report):
        if name not in self.status:
            return False
        if report.split(' | ', 1)[0] not in ('LOADING', 'READY', 'FAILED'):
            return False
        if self.status[name] == report:
            return False
        self.status[name] = report
        return True

    @property
    def pending(self):
        return [name for name, report in self.status.items() if not report.startswith('READY | ')]

    @property
    def ready(self):
        return not self.pending
