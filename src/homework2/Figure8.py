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

def update(w,h,t,pf):
    '''
    call this function in trajectory constructor to input values
    Updates f8 values
    Args:
        w (float): width
        h (float): height
        t (float): period
        pf (int): publishing frequency
    '''
    f8.update(w,h,t,pf)

def find_th0():
    '''
    Returns:
        th0 (float): turtle's starting angle
    '''
    th0 = np.arctan((2*f8.H)/f8.W)
    return th0

def find_v():
    '''
    Returns:
        vd (float list): list of velocity commands for the turtle
    '''
    vd = np.pi*np.sqrt((4*f8.H**2*np.square(np.cos(4*np.pi*f8.tlist))+f8.W**2*np.square(np.cos(2*np.pi*f8.tlist)))/(f8.T**2))
    return vd

def find_w():
    '''
    Returns:
        wd (float list): list of velocity commands for the turtle
    '''
    wd = (4*np.pi*f8.H*f8.W*(np.sin(2*np.pi*f8.tlist)*np.cos(4*np.pi*f8.tlist)-2*np.sin(4*np.pi*f8.tlist)*np.cos(2*np.pi*f8.tlist)))/(4*f8.H**2*np.square(np.cos(4*np.pi*f8.tlist))+f8.W**2*np.square(np.cos(2*np.pi*f8.tlist)))
    return wd

def main():
    update(1,2,3,4)
    th0 = find_th0()
    vd = find_v()
    wd = find_w()
    print(f8.tlist)
    print()
    print(vd)
    print()
    print(wd)


if __name__ == '__main__':
    f8 = figure8([1,1,1,1])
    #main()