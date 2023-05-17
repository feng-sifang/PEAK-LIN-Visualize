import time

import PLinApi
import PLinApi as plin_api
from ctypes import *


def get_frame_direction_as_string(direction):
    """
    Returns the string name of a PLinApi.TLINDirection value

    Parameters:
        value a PLinApi.TLINDirection value (or a number)

    Returns:
        a string name of the direction value
    """
    # check given parameter
    if isinstance(direction, plin_api.TLINDirection):
        value = direction.value
    else:
        value = int(direction)
    # translate value to string
    if value == plin_api.TLIN_DIRECTION_DISABLED.value:
        return 'Disabled'
    elif value == plin_api.TLIN_DIRECTION_PUBLISHER.value:
        return 'Publisher'
    elif value == plin_api.TLIN_DIRECTION_SUBSCRIBER.value:
        return 'Subscriber'
    elif value == plin_api.TLIN_DIRECTION_SUBSCRIBER_AUTOLENGTH.value:
        return 'Subscriber Automatic Length'


class PeakLin:
    def __init__(self):
        self.obj_plin_api = plin_api.PLinApi()
        if not self.obj_plin_api.isLoaded():
            raise Exception("PLin-API is exiting")
        self.client = plin_api.HLINCLIENT(0)
        self.hw = plin_api.HLINHW(0)
        self.hw_mode = plin_api.TLIN_HARDWAREMODE_NONE
        self.hw_baudrate = c_ushort(0)
        self.FRAME_FILTER_MASK = c_uint64(0xFFFFFFFFFFFFFFFF)
        self.mask = self.FRAME_FILTER_MASK
        self.pids = {}
        for i in range(64):
            npid = c_ubyte(i)
            self.obj_plin_api.GetPID(npid)
            self.pids[npid.value] = i

    def uninitialized(self):
        if self.client.value != plin_api.HLINCLIENT(0).value:
            self.lin_disconnect()
            self.hw = plin_api.HLINHW(0)
            self.obj_plin_api.RemoveClient(self.client)
            self.client = plin_api.HLINCLIENT(0)

    def lin_disconnect(self):
        if self.hw.value != 0:
            other_clients = False
            own_client = False
            client_size = c_ushort(255)
            clients = plin_api.HLINCLIENT * client_size.value
            lin_result = self.obj_plin_api.GetHardwareParam(
                self.hw, plin_api.TLIN_HARDWAREPARAM_CONNECTED_CLIENTS, clients, client_size)
            if lin_result == plin_api.TLIN_ERROR_OK:
                for i in range(1, client_size.value):
                    if clients[i] == 0:
                        continue
                    other_clients = other_clients | (clients[i] != self.client.value)
                    own_client = own_client | (clients[i] == self.client.value)

            if not other_clients:
                self.obj_plin_api.ResetHardwareConfig(self.client, self.hw)

            if own_client:
                lin_result = self.obj_plin_api.DisconnectClient(self.client, self.hw)
                if lin_result == plin_api.TLIN_ERROR_OK:
                    self.hw = plin_api.HLINHW(0)
                    return True

                else:
                    raise Exception("Disconnect client failed")

            else:
                return True
        # not connected
        else:
            return True

    def doLinConnect(self, hwHandle, hwMode, hwBaudrate):
        """
        Connects to LIN hardware.

        Parameters:
            hwHandle LIN hardware handle (HLINHW)
            hwMode LIN hardware mode (see TLIN_HARDWAREMODE_MASTER and TLIN_HARDWAREMODE_SLAVE)
            hwMode LIN hardware baudrate (c_ushort)

        Returns:
            True if connection is successful, False otherwise
        """
        result = False
        if self.hw.value != 0:
            # If a connection to hardware already exits,
            # disconnect this connection first.
            if not self.lin_disconnect():
                return result

        # register LIN client
        if self.client.value == 0:
            self.obj_plin_api.RegisterClient(
                "PLIN-API Console", None, self.client)

        # Try to connect the application client to the hardware with the local
        # handle.
        lin_result = self.obj_plin_api.ConnectClient(self.client, hwHandle)
        if lin_result == PLinApi.TLIN_ERROR_OK:
            # If the connection successfully assigns
            # the local handle to the member handle.
            self.hw = hwHandle
            # read hardware's parameter
            lnMode = c_int(0)
            lnCurrBaud = c_int(0)

            lin_result = self.obj_plin_api.GetHardwareParam(
                hwHandle, PLinApi.TLIN_HARDWAREPARAM_BAUDRATE, lnCurrBaud, 0)
            # check if initialization is required
            if lnMode.value == PLinApi.TLIN_HARDWAREMODE_NONE.value or lnCurrBaud.value != hwBaudrate.value:
                # Only if the current hardware is not initialized,
                # try to initialize the hardware with mode and baudrate
                lin_result = self.obj_plin_api.InitializeHardware(
                    self.client, self.hw, hwMode, hwBaudrate)
            if lin_result == PLinApi.TLIN_ERROR_OK:
                self.hw_mode = hwMode
                self.hw_baudrate = hwBaudrate
                # Set the client filter with the mask.
                self.obj_plin_api.SetClientFilter(self.client, self.hw, self.mask)
                # Read the frame table from the connected hardware.
                self.read_frame_table_from_hw()
                # Reset the last LIN error code to default.
                lin_result = PLinApi.TLIN_ERROR_OK
                result = True
            else:
                # An error occurred while initializing hardware.
                # Set the member variable to default.
                self.hw = PLinApi.HLINHW(0)
                result = False
        else:
            # The local hardware handle is invalid
            # and/or an error occurs while connecting
            # hardware with a client.
            # Set the member variable to default.
            self.hw = PLinApi.HLINHW(0)
            result = False
        if lin_result != PLinApi.TLIN_ERROR_OK:
            print(lin_result)
        return result

    def read_frame_table_from_hw(self):
        """
        Reads all values from the frame table of the hardware.

        Returns:
            A global frame List of frame entry retrieved from the hardware
        """
        # Initialize result
        result = []
        # Read each Frame Definition
        for i in range(64):
            # Before a frame entry can be read from the
            # hardware, the Frame-ID of the wanted entry
            # must be set
            frame_entry = PLinApi.TLINFrameEntry()
            frame_entry.FrameId = c_ubyte(i)
            frame_entry.ChecksumType = PLinApi.TLIN_CHECKSUMTYPE_AUTO
            frame_entry.Direction = PLinApi.TLIN_DIRECTION_SUBSCRIBER_AUTOLENGTH
            # length values are set to LIN 1.2.
            if (i >= 0x00) and (i <= 0x1F):
                frame_entry.Length = c_ubyte(2)
            elif (i >= 0x20) and (i <= 0x2F):
                frame_entry.Length = c_ubyte(4)
            elif (i >= 0x30) and (i <= 0x3F):
                frame_entry.Length = c_ubyte(8)
            # Read the information of the specified frame entry from the
            # hardware.
            lin_result = self.obj_plin_api.GetFrameEntry(
                self.hw, frame_entry)
            # Check the result value of the LinApi function call.
            if lin_result == PLinApi.TLIN_ERROR_OK:
                result.append(frame_entry)
                # Update the local application mask according to the frame
                # configuration
                if frame_entry.Direction != PLinApi.TLIN_DIRECTION_DISABLED.value:
                    llMask = c_uint64((1 << i) & self.FRAME_FILTER_MASK.value)
                    self.mask = c_uint64(self.mask.value | llMask.value)
            # If the Client and Hardware handles are valid.
            if (self.client.value != 0) and (self.hw.value != 0):
                # Set the client filter.
                self.obj_plin_api.SetClientFilter(
                    self.client, self.hw, self.mask)
        return result

    def available_hardware(self):
        hw_list = []

        count = c_ushort(0)

        available_hw = (plin_api.HLINHW * 0)()
        self.obj_plin_api.GetAvailableHardware(available_hw, 0, count)

        if count == 0:
            # use default value if either no hw is connected or an unexpected error occurred
            count = c_ushort(16)

        available_hw = (plin_api.HLINHW * count.value)()
        buff_size = c_ushort(count.value * 2)

        lin_result = self.obj_plin_api.GetAvailableHardware(available_hw, buff_size, count)
        if lin_result == plin_api.TLIN_ERROR_OK:

            # Get information for each LIN hardware found
            hw_type = c_int(0)
            device_num = c_int(0)
            channel = c_int(0)
            mode = c_int(0)

            for i in range(count.value):
                entity_num = available_hw[i]
                # Read the type of the hardware with the handle hw.
                self.obj_plin_api.GetHardwareParam(entity_num, plin_api.TLIN_HARDWAREPARAM_TYPE, hw_type, 0)

                # Read the device number of the hardware with the handle hw.
                self.obj_plin_api.GetHardwareParam(entity_num, plin_api.TLIN_HARDWAREPARAM_DEVICE_NUMBER, device_num, 0)

                # Read the channel number of the hardware with the handle hw.
                self.obj_plin_api.GetHardwareParam(entity_num, plin_api.TLIN_HARDWAREPARAM_CHANNEL_NUMBER, channel, 0)

                # Read the mode of the hardware with the handle hw (Master, Slave or None).
                self.obj_plin_api.GetHardwareParam(entity_num, plin_api.TLIN_HARDWAREPARAM_MODE, mode, 0)

                # Translate type value to string
                if hw_type.value == plin_api.LIN_HW_TYPE_USB_PRO.value:
                    name = "PCAN-USB Pro"

                elif hw_type.value == plin_api.LIN_HW_TYPE_USB_PRO_FD.value:
                    name = "PCAN-USB Pro FD"

                elif hw_type.value == plin_api.LIN_HW_TYPE_PLIN_USB.value:
                    name = "PLIN-USB"

                else:
                    name = "Unknown"

                # Add information to a result list
                hw_list.append([name, device_num.value, channel.value, entity_num])
            if len(hw_list) == 0:
                hw_list.append(["No hardware found", 0, 0, 0])
        return hw_list

    def get_formatted_rcv_msg(self, msg):
        """
        Returns a string formatted LIN received message

        Parameters:
            msg a Lin receive message (TLINRcvMsg)

        Returns:
            a string formatted LIN message
        """
        # Check if the received frame is a standard type.
        # If it is not a standard type, then ignore it.
        if msg.Type != plin_api.TLIN_MSGTYPE_STANDARD.value:
            if msg.Type == plin_api.TLIN_MSGTYPE_BUS_SLEEP.value:
                strTemp = 'Bus Sleep status message'
            elif msg.Type == plin_api.TLIN_MSGTYPE_BUS_WAKEUP.value:
                strTemp = 'Bus WakeUp status message'
            elif msg.Type == plin_api.TLIN_MSGTYPE_AUTOBAUDRATE_TIMEOUT.value:
                strTemp = 'Auto-baudrate Timeout status message'
            elif msg.Type == plin_api.TLIN_MSGTYPE_AUTOBAUDRATE_REPLY.value:
                strTemp = 'Auto-baudrate Reply status message'
            elif msg.Type == plin_api.TLIN_MSGTYPE_OVERRUN.value:
                strTemp = 'Bus Overrun status message'
            elif msg.Type == plin_api.TLIN_MSGTYPE_QUEUE_OVERRUN.value:
                strTemp = 'Queue Overrun status message'
            else:
                strTemp = 'Non standard message'
            return strTemp
        # format Data field as string
        dataStr = ""
        for i in range(msg.Length):
            dataStr = str.format("{0}{1} ", dataStr, hex(msg.Data[i]))
        # remove ending space
        dataStr = dataStr[:-1]
        # format Error field as string
        error = ""
        if msg.ErrorFlags & plin_api.TLIN_MSGERROR_CHECKSUM:
            error = error + 'Checksum,'
        if msg.ErrorFlags & plin_api.TLIN_MSGERROR_GROUND_SHORT:
            error = error + 'GroundShort,'
        if msg.ErrorFlags & plin_api.TLIN_MSGERROR_ID_PARITY_BIT_0:
            error = error + 'IdParityBit0,'
        if msg.ErrorFlags & plin_api.TLIN_MSGERROR_ID_PARITY_BIT_1:
            error = error + 'IdParityBit1,'
        if msg.ErrorFlags & plin_api.TLIN_MSGERROR_INCONSISTENT_SYNCH:
            error = error + 'InconsistentSynch,'
        if msg.ErrorFlags & plin_api.TLIN_MSGERROR_OTHER_RESPONSE:
            error = error + 'OtherResponse,'
        if msg.ErrorFlags & plin_api.TLIN_MSGERROR_SLAVE_NOT_RESPONDING:
            error = error + 'SlaveNotResponding,'
        if msg.ErrorFlags & plin_api.TLIN_MSGERROR_SLOT_DELAY:
            error = error + 'SlotDelay,'
        if msg.ErrorFlags & plin_api.TLIN_MSGERROR_TIMEOUT:
            error = error + 'Timeout,'
        if msg.ErrorFlags & plin_api.TLIN_MSGERROR_VBAT_SHORT:
            error = error + 'VBatShort,'
        if msg.ErrorFlags == 0:
            error = 'O.k. '
        # remove ending comma
        error = error[:-1]
        # format message
        return ([
            hex(self.pids[msg.FrameId]),
            msg.Length,
            dataStr,
            msg.TimeStamp,
            get_frame_direction_as_string(msg.Direction),
            error
        ])

    def hw_connect(self):
        hw_choice = plin_api.HLINHW(int(1))  # here we use the first hw temporarily
        if self.doLinConnect(hw_choice, plin_api.TLIN_HARDWAREMODE_MASTER, c_ushort(19200)):
            print("Connection successful")
        else:
            print("Connection failed")

    def read_lin_frame(self):
        rcv_msg = plin_api.TLINRcvMsg()
        lin_result = self.obj_plin_api.Read(self.client, rcv_msg)
        if lin_result == plin_api.TLIN_ERROR_OK:
            print(self.get_formatted_rcv_msg(rcv_msg))
        

if __name__ == "__main__":
    plin_app = PeakLin()
    print(plin_app.available_hardware())
    plin_app.hw_connect()
    while True:

        try:
            plin_app.read_lin_frame()

        except KeyError:
            print("KeyError")
            time.sleep(0.2)
