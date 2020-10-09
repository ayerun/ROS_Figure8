import numpy as np

class figure8(object):
    '''
    Contains figure8 parameters
    Args:
        object (list): [width (float), height (float), period (float), pub_freq (int)]
    '''

    def __init__(self,object):
        self.W = object[0]
        self.H = object[1]
        self.T = object[2]
        self.pub_freq = object[3]
        self.points = self.T*self.pub_freq              #calculate number of points in trajectory
        self.tlist = np.linspace(0,self.T,self.points)  #create a list of times

    
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
        Returns:
            th0 (float): turtle's starting angle
        '''
        th0 = np.arctan((2*self.H)/self.W)
        return th0


    def find_v(self):
        '''
        Returns:
            vd (float list): list of velocity commands for the turtle
        '''
        vd = np.pi*np.sqrt((4*self.H**2*np.square(np.cos(4*np.pi*self.tlist))+self.W**2*np.square(np.cos(2*np.pi*self.tlist)))/(self.T**2))
        return vd

    def find_w(self):
        '''
        Returns:
            wd (float list): list of velocity commands for the turtle
        '''
        wd = (4*np.pi*self.H*self.W*(np.sin(2*np.pi*self.tlist)*np.cos(4*np.pi*self.tlist)-2*np.sin(4*np.pi*self.tlist)*np.cos(2*np.pi*self.tlist)))/(4*self.H**2*np.square(np.cos(4*np.pi*self.tlist))+self.W**2*np.square(np.cos(2*np.pi*self.tlist)))
        return wd