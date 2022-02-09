import numpy as np
import cv2
import gym
import gym.spaces
from gym.utils import seeding
import pybullet as p
import pybullet_data


class Robot:
    # default values are for R2D2 model
    URDF_PATH = 'r2d2.urdf'

    # projectionMatrix settings
    CAMERA_PIXEL_WIDTH = 64  # 64 is minimum for stable-baselines
    CAMERA_PIXEL_HEIGHT = 64  # 64 is minimum for stable-baselines
    CAMERA_FOV = 90.0
    CAMERA_NEAR_PLANE = 0.01
    CAMERA_FAR_PLANE = 100.0

    # viewMatrix settings
    CAMERA_JOINT_INDEX = 14
    CAMERA_EYE_INDEX = 1
    CAMERA_UP_INDEX = 2
    CAMERA_EYE_SCALE = 0.05
    CAMERA_TARGET_SCALE = 1.0
    CAMERA_UP_SCALE = 1.0

    # for debug
    JOINT_TYPE_NAMES = ['JOINT_REVOLUTE', 'JOINT_PRISMATIC', 'JOINT_SPHERICAL', 'JOINT_PLANAR', 'JOINT_FIXED']

    def __init__(self, urdfPath=URDF_PATH, position=[0.0, 0.0, 1.0], orientation=[0.0, 0.0, 0.0, 1.0]):
        """Make a robot model.
        """
        self.urdfPath = urdfPath
        self.robotId = p.loadURDF(urdfPath, basePosition=position, baseOrientation=orientation)
        '''add useFixedBase=True to fix the robot'''
        self.projectionMatrix = p.computeProjectionMatrixFOV(self.CAMERA_FOV, float(self.CAMERA_PIXEL_WIDTH) / float(
            self.CAMERA_PIXEL_HEIGHT), self.CAMERA_NEAR_PLANE, self.CAMERA_FAR_PLANE)

    @classmethod
    def getObservationSpace(cls):
        """Return observation_space for gym Env class.
        """
        # stable-baselines3's CNN policy observation must be [0, 255] and np.uint8
        # see https://stable-baselines3.readthedocs.io/en/master/guide/custom_env.html
        # see also getObservation
        # for [0.0, 1.0] and np.float32
        # return gym.spaces.Box(low=0.0, high=1.0, shape=(Robot.CAMERA_PIXEL_HEIGHT, Robot.CAMERA_PIXEL_WIDTH, 4), dtype=np.float32)
        return gym.spaces.Box(low=0, high=255, shape=(Robot.CAMERA_PIXEL_HEIGHT, Robot.CAMERA_PIXEL_WIDTH, 4),
                              dtype=np.uint8)

    @classmethod
    def getActionSpace(cls):
        """Return action_space for gym Env class.
        """
        raise NotImplementedError

    def setAction(self, action):
        """Set action.
        """
        raise NotImplementedError

    def scaleJointVelocity(self, jointIndex, value):
        """Scale joint velocity from [-1.0, 1.0] to [-maxVelocity, maxVelocity].
        """
        # value should be from -1.0 to 1.0
        info = p.getJointInfo(self.robotId, jointIndex)
        maxVelocity = abs(info[11])
        value *= maxVelocity
        value = -maxVelocity if value < -maxVelocity else value
        value = maxVelocity if value > maxVelocity else value
        return value

    def setJointVelocity(self, jointIndex, value, force=0.0, scale=1.0):
        """Set joint velocity.
        """
        # value should be from -1.0 to 1.0
        value = self.scaleJointVelocity(jointIndex, value)
        value_1 = value * scale
        info = p.getJointInfo(self.robotId, jointIndex)
        maxForce = abs(info[10])
        p.setJointMotorControl2(bodyIndex=self.robotId,
                                jointIndex=jointIndex,
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=value_1,
                                force = maxForce)

    def scaleJointPosition(self, jointIndex, value):
        """Scale joint position from [-1.0, 1.0] to [lowerLimit, upperLimit].
        """
        # value should be from -1.0 to 1.0
        info = p.getJointInfo(self.robotId, jointIndex)
        lowerLimit = info[8]
        upperLimit = info[9]
        maxVelocity = abs(info[11])
        if lowerLimit > upperLimit:
            lowerLimit, upperLimit = upperLimit, lowerLimit  # swap
        # value *= max(abs(lowerLimit), abs(upperLimit)) # TODO: is it OK?
        # y - l = a (x - -1) = a (x + 1)
        # a = (u - l) / (1 - -1) = (u - l) / 2
        # y - l = (u - l) (x + 1) / 2
        # y = (u - l) (x + 1) * 0.5 + l
        value = (upperLimit - lowerLimit) * (value + 1.0) * 0.5 + lowerLimit
        value = lowerLimit if value < lowerLimit else value
        value = upperLimit if value > upperLimit else value
        return value, maxVelocity

    def setJointPosition(self, jointIndex, value):
        """Set joint position.
        """
        # value should be from -1.0 to 1.0
        value, maxVelocity = self.scaleJointPosition(jointIndex, value)
        p.setJointMotorControl2(bodyIndex=self.robotId,
                                jointIndex=jointIndex,
                                controlMode=p.POSITION_CONTROL,
                                maxVelocity=maxVelocity,
                                targetPosition=value)

    def invScaleJointPosition(self, jointIndex, value):
        """Return inverse joint position.
        """
        info = p.getJointInfo(self.robotId, jointIndex)
        lowerLimit = info[8]
        upperLimit = info[9]
        # y - -1 = a (x - l)
        # a = (1 - -1) / (u - l) = 2 / (u - l)
        # y - -1 = 2 (x - l) / (u - l)
        # y = 2 (x - l) / (u - l) - 1
        value = 2.0 * (value - lowerLimit) / (upperLimit - lowerLimit) - 1.0
        # value should be from -1.0 to 1.0
        value = -1.0 if value < -1.0 else value
        value = 1.0 if value > 1.0 else value
        return value

    def getJointPosition(self, jointIndex):
        """Return joint position.
        """
        jointPosition, jointVelocity, jointReactionForces, appliedJointMotorTorque = p.getJointState(self.robotId,
                                                                                                     jointIndex)
        return self.invScaleJointPosition(jointIndex, jointPosition)

    def scaleJointForce(self, jointIndex, value, i):
        """Scale joint force from [-1.0, 1.0] to [-maxForce, maxForce].
        :param i:
        """
        # value should be from -1.0 to 1.0
        info = p.getJointInfo(self.robotId, jointIndex)
        maxForce = abs(info[10])
        value *= maxForce
        value = -maxForce if value < -maxForce else value
        value = maxForce if value > maxForce else value
        return value

    def setJointForce(self, jointIndex, value, f):
        """Set joint force.
        :param f:
        """
        # value should be from -1.0 to 1.0
        value = self.scaleJointForce(jointIndex, value, 1)
        p.setJointMotorControl2(bodyIndex=self.robotId,
                                jointIndex=jointIndex,
                                controlMode=p.TORQUE_CONTROL,
                                force=value)

    def getPositionAndOrientation(self):
        """Return body's position and orientation.
        """
        return p.getBasePositionAndOrientation(self.robotId)

    def isContact(self, bodyId):
        """Return True if robot contacted with other objects.
        """
        cps = p.getContactPoints(bodyA=self.robotId, bodyB=bodyId)
        # cps = p.getContactPoints(bodyA=self.robotId, bodyB=bodyId)
        return len(cps) > 0

    def getCameraImage(self):
        """Return camera image from CAMERA_JOINT_INDEX.
        """
        # compute eye and target position for viewMatrix
        pos, orn, _, _, _, _ = p.getLinkState(self.robotId, self.CAMERA_JOINT_INDEX)
        cameraPos = np.array(pos)
        cameraMat = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3).T
        eyePos = cameraPos + self.CAMERA_EYE_SCALE * cameraMat[self.CAMERA_EYE_INDEX]
        targetPos = cameraPos + self.CAMERA_TARGET_SCALE * cameraMat[self.CAMERA_EYE_INDEX]
        up = self.CAMERA_UP_SCALE * cameraMat[self.CAMERA_UP_INDEX]
        # p.addUserDebugLine(eyePos, targetPos, lineColorRGB=[1, 0, 0], lifeTime=0.1) # red line for camera vector
        # p.addUserDebugLine(eyePos, eyePos + up * 0.5, lineColorRGB=[0, 0, 1], lifeTime=0.1) # blue line for up vector
        viewMatrix = p.computeViewMatrix(eyePos, targetPos, up)
        image = p.getCameraImage(self.CAMERA_PIXEL_WIDTH, self.CAMERA_PIXEL_HEIGHT, viewMatrix, self.projectionMatrix,
                                 shadow=1, lightDirection=[1, 1, 1], renderer=p.ER_BULLET_HARDWARE_OPENGL)
        return image

    def getObservation(self):
        """Return RGB and depth images from CAMERA_JOINT_INDEX.
        """
        width, height, rgbPixels, depthPixels, segmentationMaskBuffer = self.getCameraImage()
        rgba = np.array(rgbPixels, dtype=np.float32).reshape((height, width, 4))
        depth = np.array(depthPixels, dtype=np.float32).reshape((height, width, 1))
        # seg = np.array(segmentationMaskBuffer, dtype=np.float32).reshape((height, width, 1))
        rgb = np.delete(rgba, [3], axis=2)  # delete alpha channel
        rgb01 = np.clip(rgb * 0.00392156862, 0.0, 1.0)  # rgb / 255.0, normalize
        obs = np.insert(rgb01, [3], np.clip(depth, 0.0, 1.0), axis=2)
        # obs = np.insert(obs, [4], seg, axis=2) # TODO: normalize
        # stable-baselines3's CNN policy observation must be [0, 255] and np.uint8
        # see https://stable-baselines3.readthedocs.io/en/master/guide/custom_env.html
        # see also getObservationSpace
        obs = np.array(np.clip(obs * 255, 0, 255), dtype=np.uint8)
        return obs

    def printJointInfo(self, index):
        """Print joint information.
        """
        jointIndex, jointName, jointType, qIndex, uIndex, flags, jointDamping, jointFriction, jointLowerLimit, jointUpperLimit, jointMaxForce, jointMaxVelocity, linkName, jointAxis, parentFramePos, parentFrameOrn, parentIndex = p.getJointInfo(
            self.robotId, index)
        line = [jointName.decode('ascii'), '\n\tjointIndex\t', jointIndex, '\n\tjointName\t', jointName,
                '\n\tjointType\t', jointType, '\t', self.JOINT_TYPE_NAMES[jointType], '\n\tqIndex\t', qIndex,
                '\n\tuIndex\t', uIndex, '\n\tflags\t', flags, '\n\tjointDamping\t', jointDamping, '\n\tjointFriction\t',
                jointFriction, '\n\tjointLowerLimit\t', jointLowerLimit, '\n\tjointUpperLimit\t', jointUpperLimit,
                '\n\tjointMaxForce\t', jointMaxForce, '\n\tjointMaxVelocity\t', jointMaxVelocity, '\n\tlinkName\t',
                linkName, '\n\tjointAxis\t', jointAxis, '\n\tparentFramePos\t', parentFramePos, '\n\tparentFrameOrn\t',
                parentFrameOrn, '\n\tparentIndex\t', parentIndex, '\n']
        print(''.join([str(item) for item in line]))
        # line = [ jointIndex, jointName.decode('ascii'), self.JOINT_TYPE_NAMES[jointType] ]
        # print('\t'.join([ str(item) for item in line ]))

    def printJointInfoArray(self, indexArray):
        """Print joint informations.
        """
        for index in indexArray:
            self.printJointInfo(index)

    def printAllJointInfo(self):
        """Print all joint informations.
        """
        self.printJointInfoArray(range(p.getNumJoints(self.robotId)))


class HSR_G(Robot):
    URDF_PATH = 'hsrb4s.urdf'

    # viewMatrix settings
    CAMERA_JOINT_INDEX = 19
    CAMERA_EYE_INDEX = 2
    CAMERA_UP_INDEX = 1
    CAMERA_EYE_SCALE = 0.01
    CAMERA_TARGET_SCALE = 1.0
    CAMERA_UP_SCALE = -1.0

    def __init__(self, urdfPath=URDF_PATH, position=[0.0, 0.0, 0.05], orientation=[0.0, 0.0, 0.0, 1.0]):
        """Make a HSR robot model.
        """
        super(HSR_G, self).__init__(urdfPath, position, orientation)

    # override methods
    @classmethod
    def getActionSpace(cls):
        """Return action_space for gym Env class.
        """
        n = 13
        low = -1.0 * np.ones(n)
        high = 1.0 * np.ones(n)
        return gym.spaces.Box(low=low, high=high, dtype=np.float32)

    def setAction(self, action):
        """Set action.
        """
        self.setWheelVelocity(action[0], action[1])
        self.setBaseRollVelocity(action[2])
        self.setTorsoLiftPosition(action[3])
        self.setHeadPosition(action[4], action[5])
        self.setArmPosition(action[6], action[7], action[8])
        self.setWristVelocity(action[9], action[10])
        # self.setHandPosition(action[11], action[12], action[13], action[14], action[15], action[16], action[17], action[18], action[19]) # TODO
        self.setGripperVelocity(action[11], action[12])

    def isContact(self, bodyId):
        """Return True if HSR contacted with other objects.
        """
        # cps = p.getContactPoints(bodyA=self.robotId, bodyB=bodyId, linkIndexA=27) # only for wrist_roll_link
        # cps = p.getContactPoints(bodyA=self.robotId, bodyB=bodyId)
        cps = p.getContactPoints(bodyA=self.robotId, bodyB=bodyId)
        return len(cps) > 0

    # HSR specific methods
    def setWheelVelocity(self, left, right):
        """Set wheel's velocity.
        """
        self.setJointVelocity(2, right, 1)
        self.setJointVelocity(3, left, 1)

    def setWheelPosition(self, left, right):
        """Set wheel's velocity.
        """
        self.setJointPosition(2, right)
        self.setJointPosition(3, left)

    def setBaseRollVelocity(self, roll):
        """Set base roll position.
        """
        self.setJointVelocity(1, roll, 0.5)

    def setBaseRollPosition(self, roll):
        """Set base roll position.
        """
        self.setJointPosition(1, roll)

    def setTorsoLiftVelocity(self, lift):
        """Set wheel's velocity.
        """
        self.setJointVelocity(12, lift, 0.5)

    def setTorsoLiftPosition(self, lift):
        """Set torso lift position.
        """
        self.setJointPosition(12, lift)

    def setHeadVelocity(self, pan, tilt):
        """Set wheel's velocity.
        """
        self.setJointVelocity(13, pan, 1)
        self.setJointVelocity(14, tilt, 1)

    def setHeadPosition(self, pan, tilt):
        """Set head position.
        """
        self.setJointPosition(13, pan)
        self.setJointPosition(14, tilt)

    def setArmVelocity(self, lift, flex, roll):
        """Set wheel's velocity.
        """
        self.setJointVelocity(23, lift, 0.1)
        self.setJointVelocity(24, flex, 0.5)
        self.setJointVelocity(25, roll, 0.5)

    def setArmPosition(self, lift, flex, roll):
        """Set arm position.
        """
        self.setJointPosition(23, lift)
        self.setJointPosition(24, flex)
        self.setJointPosition(25, roll)

    def setWristVelocity(self, flex, roll):
        """Set wheel's velocity.
        """
        self.setJointVelocity(26, flex, 0.5)
        self.setJointVelocity(27, roll, 0.5)

    def setWristPosition(self, flex, roll):
        """Set wrist position.
        """
        self.setJointPosition(26, flex)
        self.setJointPosition(27, roll)

    # def setHandPosition(self, motor, leftProximal, leftSpringProximal, leftMimicDistal, leftDistal, rightProximal, rightSpringProximal, rightMimicDistal, rightDistal): # TODO
    #     """Set hand position.
    #     """
    #     self.setJointPosition(30, motor)
    #     self.setJointPosition(31, leftProximal)
    #     self.setJointPosition(32, leftSpringProximal)
    #     self.setJointPosition(33, leftMimicDistal)
    #     self.setJointPosition(34, leftDistal)
    #     self.setJointPosition(37, rightProximal)
    #     self.setJointPosition(38, rightSpringProximal)
    #     self.setJointPosition(39, rightMimicDistal)
    #     self.setJointPosition(40, rightDistal)

    def setGripperVelocity(self, left, right):
        """Set wheel's velocity.
        """
        self.setJointVelocity(30, left, 0.5, 1)
        self.setJointVelocity(32, right, 0.5, 1)

    def setGripperPosition(self, left, right):
        """Set gripper position.
        """
        self.setJointPosition(30, left)
        self.setJointPosition(32, right)

    def getWheelPosition(self):
        """Set wheel's velocity.
        """
        return self.getJointPosition(2), self.getJointPosition(3)

    def getBaseRollPosition(self):
        """Get base roll position.
        """
        return self.getJointPosition(1)

    def getTorsoLiftPosition(self):
        """Get torso lift position.
        """
        return self.getJointPosition(12)

    def getHeadPosition(self):
        """Get head position.
        """
        return self.getJointPosition(13), self.getJointPosition(14)

    def getArmPosition(self):
        """Get arm position.
        """
        return self.getJointPosition(23), self.getJointPosition(24), self.getJointPosition(25)

    def getWristPosition(self):
        """Get wrist position.
        """
        return self.getJointPosition(26), self.getJointPosition(27)

    def getGripperPosition(self):
        """Get gripper position.
        """
        return self.getJointPosition(30), self.getJointPosition(32)


class HSRSimple_G(HSR_G):
    @classmethod
    def getActionSpace(cls):
        """Return action_space for gym Env class.
        """
        n = 13
        low = -1.0 * np.ones(n)
        high = 1.0 * np.ones(n)
        return gym.spaces.Box(low=low, high=high, dtype=np.float32)

    def setAction(self, action):
        """Set action.
        """
        self.setWheelPosition(0, 0)
        self.setBaseRollPosition(0)
        self.setTorsoLiftPosition(0.5)
        self.setHeadPosition(0, -1)
        self.setArmPosition(action[6], -1, 0)
        self.setWristPosition(0.5, 0)
        self.setGripperVelocity(action[11], action[12])


class GraspingEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    GRAVITY = -10.0
    BULLET_STEPS = 120  # p.setTimeStep(1.0 / 240.0), so 1 gym step == 0.5 sec.

    def __init__(self, render=False, robot_model=HSRSimple_G, max_steps=500, num_foods=3, num_fakes=0, object_size=1.0,
                 object_radius_scale=1.0, object_radius_offset=1.0, object_angle_scale=1.0):
        """Initialize environment.
        """
        ### gym variables
        self.observation_space = robot_model.getObservationSpace()  # classmethod
        self.action_space = robot_model.getActionSpace()  # classmethod
        self.reward_range = (-1.0, 1.0)
        self.seed()
        ### pybullet settings
        self.render = render
        self.physicsClient = p.connect(p.GUI if render else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        ### env variables
        self.robot_model = robot_model
        self.max_steps = max_steps
        self.num_foods = num_foods
        self.num_fakes = num_fakes
        self.object_size = object_size
        self.object_radius_scale = object_radius_scale
        self.object_radius_offset = object_radius_offset
        self.object_angle_scale = object_angle_scale
        self.plane_id = None
        self.robot = None
        self.object_ids = []
        self.single_object_id = []
        ### episode variables
        self.steps = 0
        self.episode_rewards = 0.0

    def close(self):
        """Close environment.
        """
        p.disconnect(self.physicsClient)

    def reset(self):
        """Reset environment.
        """
        self.steps = 0
        self.episode_rewards = 0
        p.resetSimulation()
        # p.setTimeStep(1.0 / 240.0)
        p.setGravity(0, 0, self.GRAVITY)
        self.plane_id = p.loadURDF('plane.urdf')
        self.robot = self.robot_model()
        self.object_ids = []


        object_id = p.loadURDF('food_cube_for_grasping.urdf', [ 0.5, 0.1, 0.0], globalScaling=3)
        self.object_ids.append(object_id)

        for i in range(self.BULLET_STEPS):
            p.stepSimulation()
            p.resetDebugVisualizerCamera(
                cameraDistance=3,
                cameraYaw=30,
                cameraPitch=0,
                cameraTargetPosition=[0, 0.5, 1])
        obs = self._getObservation()
        # self.robot.printAllJointInfo()
        return obs

    def step(self, action):
        """Apply action to environment, then return observation and reward.
        """
        self.steps += 1
        self.robot.setAction(action)
        reward = -10.0 * float(self.num_foods) / float(self.max_steps)  # so agent needs to eat foods quickly
        for i in range(self.BULLET_STEPS):
            p.stepSimulation()
            reward += self._getReward()
        self.episode_rewards += reward
        obs = self._getObservation()
        done = self._isDone()
        pos, orn = self.robot.getPositionAndOrientation()
        info = {'steps': self.steps, 'pos': pos, 'orn': orn}
        if done:
            info['episode'] = {'r': self.episode_rewards, 'l': self.steps}
            # print(self.episode_rewards, self.steps)
        # print(self.robot.getBaseRollPosition(), self.robot.getTorsoLiftPosition(), self.robot.getHeadPosition(), self.robot.getArmPosition(), self.robot.getWristPosition(), self.robot.getGripperPosition()) # for HSR debug
        # print(self.robot.getHeadPosition(), self.robot.getGripperPosition()) # for R2D2 debug
        return obs, reward, done, info

    def render(self, mode='human', close=False):
        """This is a dummy function. This environment cannot control rendering timing.
        """
        pass

    def seed(self, seed=None):
        """Set random seed.
        """
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _getReward(self):
        """Detect contact points and return reward.
        """
        reward = 0
        objectID = [object_id for object_id in self.object_ids ]

        for object_id in objectID:
            pos, _ = p.getBasePositionAndOrientation(object_id)
            if pos[2] > 0.2:
                reward += 10
                p.removeBody(object_id)
                self.object_ids.remove(object_id)

        '''
        for object_id in objectID:
            closestPoints_left = p.getClosestPoints(object_id, self.robot.robotId, 0.1, -1, 31)
            closestPoints_right = p.getClosestPoints(object_id, self.robot.robotId, 0.1, -1, 33)
            closestPoints_cross = p.getClosestPoints(self.robot.robotId, self.robot.robotId, 0.01, 31, 33)
            pos, _ = p.getBasePositionAndOrientation(object_id)
            numPt_left = len(closestPoints_left)
            numPt_right = len(closestPoints_right)
            numPt_cross = len(closestPoints_cross)
            # print(numPt)
            if (numPt_left > 0) and (numPt_right > 0):
                # print("reward:")
                reward = 0.5 * (closestPoints_left[0][8] + closestPoints_right[0][8]) * 0.01

            if (numPt_cross > 0):
                reward = - closestPoints_cross[0][8] * 0.01
            if pos[2] > 0.22:
                reward += 10
                p.removeBody(object_id)
                self.object_ids.remove(object_id)

        contacted_object_ids = [ object_id for object_id in self.object_ids if self.robot.isContact(object_id) ]

        for object_id in contacted_object_ids:
            reward += 1 if self._isFood(object_id) else -1
            p.removeBody(object_id)
            self.object_ids.remove(object_id)
        '''
        return reward

    def _getObservation(self):
        """Get observation.
        """
        obs = self.robot.getObservation()
        return obs

    def _isFood(self, object_id):
        """Check if object_id is a food.
        """
        baseLink, urdfPath = p.getBodyInfo(object_id)
        return urdfPath == b'food_cube.urdf'  # otherwise, fake

    def _isDone(self):
        """Check if episode is done.
        """
        available_object_ids = [object_id for object_id in self.object_ids if self._isFood(object_id)]
        return self.steps >= self.max_steps or len(available_object_ids) <= 0
