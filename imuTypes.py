class Orientation:
    def __init__(self, x=0., y=0., z=0., w=0., covariance=None):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
        self.covariance = covariance

    def toString(self):
        return "Orientation -> x : " + str(self.x) + " y : " + str(self.y) + " z : " + str(self.z) + " w : " + str(self.w)


class Velocity:
    def __init__(self, x=0., y=0., z=0.):
        self.x = x
        self.y = y
        self.z = z

    def toString(self):
        return "Velocity -> x : " + str(self.x) + " y : " + str(self.y) + " z : " + str(self.z)


class Acceleration:
    def __init__(self, x=0., y=0., z=0.):
        self.x = x
        self.y = y
        self.z = z

    def toString(self):
        return "Acceleration -> x : " + str(self.x) + " y : " + str(self.y) + " z : " + str(self.z)


class Header:
    def __init__(self, seq=0, secs=0., nsecs=0., frameID=None):
        self.seq= seq
        self.secs= secs
        self.nsecs= nsecs
        self.frameID= frameID

    def toString(self):
        return "Header -> secs : " + str(self.secs) + " seq : " + str(self.seq) + " nsecs : " + str(self.nsecs)


class IMU:
    def __init__(self):
        self.header = Header()
        self.orientation = Orientation()
        self.angularVelocity = Velocity()
        self.linearAcceleration = Acceleration()

