# Old point_home using PID

# PID Control for finding bearing home
        self.L1 = 1
        self.L2 = 0
        self.L3 = 0
        self.turn_v = 0
        self.old_turn_errors = [0]

def point_home(self):
        """
        Turn on the spot until pointing at home.
        """
        # Positive error = turn right
        tan_bit = math.tan((self.theta - self.phi) / 2)

        try:
            exp_bit = math.exp(-tan_bit)
        except OverflowError:
            exp_bit = 0

        error = (1.0 / (1.0 + exp_bit)) - 0.5

        last = self.old_turn_errors[-1]
        diff = error - last
        total = sum(self.old_turn_errors)
        
        self.old_turn_errors.append(error)

        delta_turn_v = self.L1 * error + self.L2 * total + self.L3 * diff
        self.turn_v = self.turn_v + delta_turn_v

        turn(self.connection, self.turn_v, - self.turn_v)
        print 'error: ' + str(error)
        print 'turn_v: ' + str(self.turn_v)