



class WaveformMatcher():
    def __init__(self, waveform):
        self.waveform = waveform
        self.waveform_length = len(waveform)

    def match(self, waveform) -> float:
        # Returns the similarity metric for the two waveforms.abs
        waveform[-self.waveform_length:]





class VelocityAssessor():
    # Implements a state machine

    def __init__(self, name):
        self.THRESHOLD = 0.025
        self.prev_component = 0
        self.state = 0
        self.name = name

    def assess(self, velocity_component):
        ret = 0

        if self.state == 0 and abs(velocity_component) > self.THRESHOLD:
            self.state = 1 # Waiting for peak

        elif self.state == 1 and abs(velocity_component) < abs(self.prev_component):
            self.peak_component = self.prev_component
            self.state = 2 # Found peak, waiting for reverse or something

        elif self.state == 2 and velocity_component * self.peak_component < 0:
            self.state = 3 # Crossed zero

        elif self.state == 3 and abs(velocity_component) < abs(self.prev_component):
            self.state = 4 # Found opposite peak, wait for under thresh
            negative_motion = self.peak_component < 0
            ret = -1 if negative_motion else 1
            print(f"Saw a motion in the {'negative' if negative_motion else 'positive'} {self.name} direction.")

        elif self.state == 4 and abs(velocity_component) < self.THRESHOLD:
            self.state = 0

        self.prev_component = velocity_component

        return ret
