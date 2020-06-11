import time, epics
from sardana import State
from sardana.pool.controller import MotorController
from sardana.pool.controller import Type, Description, DefaultValue


class EpicsMotorController(MotorController):
    ctrl_properties = {'setpoint': {Type: str,
                                Description: 'The PV for the setpoint',
                                DefaultValue: 'HHG:MOTOR:SETPOINT'},
                       'readback': {Type: str,
                                Description: 'The PV for the readback',
                                DefaultValue: 'HHG:MOTOR:READBACK'}}
    
    MaxDevice = 1
    
    def __init__(self, inst, props, *args, **kwargs):
        super(EPICSmotorController, self).__init__(
            inst, props, *args, **kwargs)

        
        
        print('EPICS Motor Controller Initialization ...'),
        print('SUCCESS')
        # do some initialization
        self._motors = {}
        self._isMoving = None
        self._moveStartTime = None
        self._threshold = 0.05
        self._target = None
        self._timeout = 10

    def AddDevice(self, axis):
        self._motors[axis] = True

    def DeleteDevice(self, axis):
        del self._motors[axis]

    StateMap = {
        1: State.On,
        2: State.Moving,
        3: State.Fault,
    }

    def StateOne(self, axis):
        limit_switches = MotorController.NoLimitSwitch     
        pos = self.ReadOne(axis)
        now = time.time()
        
        try:
            if self._isMoving == False:
                state = State.On
            elif self._isMoving & (abs(pos-self._target) > self._threshold): 
                # moving and not in threshold window
                if (now-self._moveStartTime) < self._timeout:
                    # before timeout
                    state = State.Moving
                else:
                    # after timeout
                    self._log.warning('EPICS Motor Timeout')
                    self._isMoving = False
                    state = State.On
            elif self._isMoving & (abs(pos-self._target) <= self._threshold): 
                # moving and within threshold window
                self._isMoving = False
                state = State.On
                #print('Kepco Tagret: %f Kepco Current Pos: %f' % (self._target, pos))
            else:
                state = State.Fault
        except:
            state = State.Fault
        
        return state, 'EPICS Motor', limit_switches  

    def ReadOne(self, axis):
        return float(epics.caget(self.readback))
        
    def StartOne(self, axis, position):
        self._moveStartTime = time.time()
        self._isMoving = True
        self._target = position
        epics.caput(self.setpoint, position)

    def StopOne(self, axis):
        pass

    def AbortOne(self, axis):
        pass

    