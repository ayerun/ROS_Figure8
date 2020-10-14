import numpy as np

class figure8(object):
    '''
    Class to calculate details of a figure eight trajectory.
    Contains functions to map figure eight trajectory to turtlebot control input.
    This code does not directly interact with any hardware.
    Args:
        object (list): [width (float), height (float), period (float), pub_freq (int)]
    '''

    def __init__(self,object):
        '''
        Constructor
        Initaializes parameters of figure eight trajectory
        Arg:
            object (list): [width (float), height (float), period (float), pub_freq (int)]
        '''
        self.W = object[0]
        self.H = object[1]
        self.T = object[2]
        self.pub_freq = object[3]
        self.points = self.T*self.pub_freq              #calculate number of points in trajectory
        self.tlist = np.linspace(0,self.T,self.points)  #create a list of times for simulation
        self.plist = np.linspace(0,self.T,100)         #create a list of times for path display
    
    def update(self,w,h,t,pf):
        '''
        Updates class values
        Args:
            w (float): width
            h (float): height
            t (float): period
            pf (int): publishing frequency
        '''
        self.W = w
        self.H = h
        self.T = t
        self.pub_freq = pf
        self.points = self.T*self.pub_freq
        self.tlist = np.linspace(0,self.T,self.points)

    def find_th0(self):
        '''
        Calculate initial angle for figure8
        Returns:
            th0 (float): turtle's starting angle
        '''
        th0 = np.arctan((2*self.H)/self.W)
        return th0


    def find_vlist(self):
        '''
        Calculates list of velocity values that can be used in turtle commands
        Returns:
            vd (float list): list of velocity commands for the turtle
        '''
        vd = np.pi*np.sqrt((4*self.H**2*np.square(np.cos((4*np.pi*self.tlist)/self.T))+self.W**2*np.square(np.cos((2*np.pi*self.tlist)/self.T)))/(self.T**2))
        return vd

    def find_wlist(self):
        '''
        Calculates list of angular velocity values that can be used in turtle commands
        Returns:
            wd (float list): list of velocity commands for the turtle
        '''
        wd = (4*np.pi*self.H*self.W*(np.sin((2*np.pi*self.tlist)/self.T)*np.cos((4*np.pi*self.tlist)/self.T)-2*np.sin((4*np.pi*self.tlist)/self.T)*np.cos((2*np.pi*self.tlist)/self.T)))/(self.T*(4*self.H**2*np.square(np.cos((4*np.pi*self.tlist)/self.T))+self.W**2*np.square(np.cos((2*np.pi*self.tlist)/self.T))))
        return wd
    
    def find_xlist(self):
        '''
        Calculates all x positions of the trajectory
        Returns:
            xlist (float list): x positions of trajectory
        '''
        xlist = (self.W*np.sin((2*np.pi*self.plist)/self.T))/2
        xlist = xlist.tolist()
        return xlist

    def find_ylist(self):
        '''
        Calculates all y positions of the trajectory
        Returns:
            ylist (float list): y positions of trajectory
        '''
        ylist = (self.H*np.sin((4*np.pi*self.plist)/self.T))/2
        ylist = ylist.tolist()
        return ylist

    def find_thlist(self):
        '''
        Calculates all turtle angles in the trajectory
        Returns:
            thlist (float list): angles of trajectory
        '''
        thlist = np.arctan((2*self.H*np.cos((4*np.pi*self.plist)/self.T))/(self.W*np.cos((2*np.pi*self.plist)/self.T)))
        return thlist

    def find_xd(self,t):
        '''
        Calculates desired x position at time t
        Args:
            t (float): time
        Returns:
            xd (float): desired x position at time t
        '''
        xd = (self.W*np.sin((2*np.pi*t)/self.T))/2
        return xd

    def find_yd(self,t):
        '''
        Calculates desired y position at time t
        Args:
            t (float): time
        Returns:
            yd (float): desired y position at time t
        '''
        yd = (self.H*np.sin((4*np.pi*t)/self.T))/2
        return yd

    def find_xd_dot(self,t):
        '''
        Calculates desired x velocity at time t
        Args:
            t (float): time
        Returns:
            xd_dot (float): desired x velocity at time t
        '''
        xd_dot = (np.pi*self.W*np.cos((2*np.pi*t)/self.T))/self.T
        return xd_dot

    def find_yd_dot(self,t):
        '''
        Calculates desired y velocity at time t
        Args:
            t (float): time
        Returns:
            yd_dot (float): desired y velocity at time t
        '''
        yd_dot = (2*np.pi*self.H*np.cos((4*np.pi*t)/self.T))/self.T
        return yd_dot

    def find_xd_ddot(self,t):
        '''
        Calculates desired x acceleration at time t
        Args:
            t (float): time
        Returns:
            xd_ddot (float): desired x acceleration at time t
        '''
        xd_ddot = (-2*np.pi**2*self.W*np.sin((2*np.pi*t)/self.T))/self.T**2
        return xd_ddot

    def find_yd_ddot(self,t):
        '''
        Calculates desired y acceleration at time t
        Args:
            t (float): time
        Returns:
            yd_ddot (float): desired x acceleration at time t
        '''
        yd_ddot = (-8*np.pi**2*self.H*np.sin((4*np.pi*t)/self.T))/self.T**2
        return yd_ddot

    def find_vd(self,t):
        '''
        Calculates desired velocity at time t
        Args:
            t (float): time
        Returns:
            vd (float): velocity at time t
        '''
        vd = np.pi*np.sqrt((4*self.H**2*np.square(np.cos((4*np.pi*t)/self.T))+self.W**2*np.square(np.cos((2*np.pi*t)/self.T)))/(self.T**2))
        return vd

    def find_wd(self,t):
        '''
        Calculates desired angular velocity at time t
        Args:
            t (float): time
        Returns:
            wd (float): angular velocity at time t
        '''
        wd = (4*np.pi*self.H*self.W*(np.sin((2*np.pi*t)/self.T)*np.cos((4*np.pi*t)/self.T)-2*np.sin((4*np.pi*t)/self.T)*np.cos((2*np.pi*t)/self.T)))/(self.T*(4*self.H**2*np.square(np.cos((4*np.pi*t)/self.T))+self.W**2*np.square(np.cos((2*np.pi*t)/self.T))))
        return wd