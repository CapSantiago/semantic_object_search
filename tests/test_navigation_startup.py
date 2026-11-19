from types import SimpleNamespace

import pytest


pytest.importorskip('rclpy')
import initialize_sim_amcl as startup


def test_dropped_lifecycle_reply_is_retried_within_startup_deadline(monkeypatch):
    now = [0.]
    requests = []
    removed = []

    class Request:
        def __init__(self, complete):
            self.complete = complete
            self.canceled = False

        def done(self):
            return self.complete

        def result(self):
            return SimpleNamespace(current_state=SimpleNamespace(id=3))

        def cancel(self):
            self.canceled = True

    def spin(*args, **kwargs):
        now[0] += .1

    def submit(_):
        response = Request(bool(requests))
        requests.append(response)
        return response

    client = SimpleNamespace(service_is_ready=lambda: True,
                             call_async=submit, remove_pending_request=removed.append)
    monkeypatch.setattr(startup, 'time', SimpleNamespace(monotonic=lambda: now[0]))
    monkeypatch.setattr(startup.rclpy, 'spin_once', spin)
    startup.wait_until_active(None, client, lambda: True, 5., 'startup failed')
    assert len(requests)==2 and removed==requests[:1]
    assert requests[0].canceled and now[0]<5.
