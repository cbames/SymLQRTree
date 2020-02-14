class PendulumItem(object):
    def __init__(self, coords, data):
        self.coords = coords
        self.data = data

    def __len__(self):
        return len(self.coords)

    def __getitem__(self, i):
        return self.coords[i]

    def __repr__(self):
        return 'Item({}, {}, {})'.format(self.coords[0], self.coords[1], self.data)


class LinearizationPoint(object):

    def __init__(self,K, # Feedback term
                      S, # Value function 
                      A, # passive dynamics
                      B, # control dynamics
                      epsH, # level set of lyapuanov 
                      Vx, # spatial derivative of value function
                      Qval, # action value function 
                      X0, # cooridate of linearization 
                      T, # ??
                      U0) # Command from trajectory if being linearized around trajectory 

        self.K = K 
        self.S = S 
        self.A = A 
        self.B = B 
        self.epsH = self.epsH
        self.Vx = Vx 
        self.Qval = Qval 
        self.X0 = X0 
        self.T = T
        self.U0 = U0