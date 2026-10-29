"""The Hypothesis profile every property test runs under."""

from hypothesis import settings

# Every property test runs the same examples on every host: an example takes
# as long as the host needs (no deadline), each test draws from a seed fixed
# per test, and no example database carries over from one run to the next.
settings.register_profile("control_core_py", deadline=None, derandomize=True, database=None)
settings.load_profile("control_core_py")
