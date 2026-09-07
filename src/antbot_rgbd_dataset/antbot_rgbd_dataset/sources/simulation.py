"""Optional simulation adapter with no Isaac/Gazebo import."""


class SimulationRGBDSource:
    """Wrap any source already producing unified RGBDFrame values."""

    def __init__(self, source):
        self.source = source

    def start(self):
        return self.source.start()

    def read(self):
        frame = self.source.read()
        if frame is not None:
            frame.validate()
        return frame

    def stop(self):
        return self.source.stop()
