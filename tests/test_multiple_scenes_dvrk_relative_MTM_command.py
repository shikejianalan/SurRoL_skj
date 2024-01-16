# load and define the MTM
from dvrk import mtm
import crtk
import numpy

# ral = crtk.ral('mtm_node')
# m = mtm(ral, 'MTML')
# ral.check_connections()
# ral.spin()
m = mtm('MTML')

# When True, force direction is constant.  Otherwise force direction defined in gripper coordinate system
m.body_set_cf_orientation_absolute(True)

# about 2N force in y direction
m.body.servo_cf(numpy.array([0.0, 0.0, 2.0, 0.0, 0.0, 0.0]))

# lock the MTM wrist orientation
m.lock_orientation_as_is()

# turn gravity compensation on/off
m.use_gravity_compensation(True)

# turn off forces
# self.arm.body.servo_cf(numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
m.body.servo_cf(numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))