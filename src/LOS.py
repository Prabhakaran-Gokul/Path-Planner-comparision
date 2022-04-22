class LOS():
    def __init__(self):
        self.Dl = self.Ds = self.l = self.s = self.lf = self.sf = None
        self.ds = self.dl = self.abs_Dl = self.d_esl = self.esl = None
        self.coord_func = None
        self.curr = None

    def swap(self, l, s):
        return s, l

    def no_swap(self, l, s):
        return l, s

    def reset(self, src, tgt):
        Di = tgt[0] - src[0]
        Dj = tgt[1] - src[1] 
        if abs(Di) > abs(Dj):
            self.Dl = Di 
            self.Ds = Dj 
            self.l = src[0]
            self.s = src[1]
            self.lf = tgt[0]
            self.sf = tgt[1]
            self.coord_func = lambda l, s: self.no_swap(l, s)

        else:
            self.Dl = Dj
            self.Ds = Di
            self.l = src[1]
            self.s = src[0]
            self.lf = tgt[1]
            self.sf = tgt[0]
            self.coord_func = lambda l, s: self.swap(l, s)

        self.ds = self.sign(self.Ds)
        self.dl = self.sign(self.Dl)
        self.abs_Dl = abs(self.Dl)
        self.d_esl = abs(self.Dl) * self.ds
        self.esl = 0

    def sign(self, value):
        if value > 0:
            return 1
        elif value < 0:
            return -1
        else:
            return 0

    def next(self):
        self.l += self.dl
        self.esl += self.Ds
        if 2 * abs(self.esl) >= self.abs_Dl:
            self.esl -= self.d_esl
            self.s += self.ds

        self.curr = self.coord_func(self.l, self.s)
        return self.curr

    def get(self, src, tgt):
        line = [src]
        self.reset(src, tgt)
        while (self.l != self.lf and self.s != self.sf):
            line.append(self.next())

        return line

# los = LOS()
# print(los.get((2,2), (4,5)))