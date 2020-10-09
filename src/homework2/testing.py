    def linspace(self):
        '''
        creates a list of time values based on
        the publishing frequency and period
        Returns:
            list of time values
        '''
        step = self.T/self.points
        rlist = []
        for i in range(0,self.points):
            rlist.append(0+i*step)
        return rlist