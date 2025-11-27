import os
from gz.sim import TestFixture

home = os.getenv("HOME")
fixture = TestFixture(f'{home}/sitl_models/Gazebo/worlds/vtail_runway.sdf')
fixture.finalize()

server = fixture.server()
server.run(True, 0, False)

