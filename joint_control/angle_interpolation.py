'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import rightBackToStand


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
	self.startTime = None

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
	if(self.startTime == None):
            self.startTime = perception.time
        curr_time = perception.time - self.startTime

        names = keyframes[0]
	times = keyframes[1]
	keys  = keyframes[2]
        
        for i in range(len(names)):   
            time = times[i]    
            if names[i] in self.joint_names:              
		    for j in range(len(time)-1):
			maxTime = time[j+1]
			minTime = time[j]
			if (minTime <= curr_time and curr_time <= maxTime): 
			    t = (curr_time - minTime ) / (maxTime- minTime)
			    p0 = keys[i][j][0]
			    p1 = p0 + keys[i][j][2][2]
			    p3 = keys[i][j+1][0]
			    p2 = p3 + keys[i][j+1][1][2]  
			    angle = ((1 - t)**3)* p0 + 3*t *((1 - t)**2) * p1 + 3*(t**2) * (1-t) * p2 + (t**3) * p3
			    target_joints[names[i]] = angle
			    if (names[i] == "LHipYawPitch"):
			        target_joints["RHipYawPitch"] = angle
			    
			    
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = rightBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
