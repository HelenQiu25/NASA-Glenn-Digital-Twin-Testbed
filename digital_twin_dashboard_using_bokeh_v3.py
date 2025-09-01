"""
App to read in varying voltages from Arduino.

To serve the app, run

    bokeh serve --show potentiometer_app.py

on the command line.
"""

import asyncio
import math
import re
import sys
import time

import numpy as np
import pandas as pd

import serial
import serial.tools.list_ports

import bokeh.plotting
import bokeh.io
import bokeh.layouts
import bokeh.driving
from bokeh.models import TextInput
from bokeh.models import Slider

def find_arduino(port=None):
    """Get the name of the port that is connected to Arduino."""
    if port is None:
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if p.manufacturer is not None and "Arduino" in p.manufacturer:
                port = p.device
    return port


def handshake_arduino(
    arduino, sleep_time=1, print_handshake_message=False, handshake_code=0
):
    """Make sure connection is established by sending
    and receiving bytes."""
    # Close and reopen
    arduino.close()
    arduino.open()

    # Chill out while everything gets set
    time.sleep(sleep_time)

    # Set a long timeout to complete handshake
    timeout = arduino.timeout
    arduino.timeout = 2

    # Read and discard everything that may be in the input buffer
    _ = arduino.read_all()

    # Send request to Arduino
    arduino.write(bytes([handshake_code]))

    # Read in what Arduino sent
    handshake_message = arduino.read_until()

    # Send and receive request again
    arduino.write(bytes([handshake_code]))
    handshake_message = arduino.read_until()

    # Print the handshake message, if desired
    if print_handshake_message:
        print("Handshake message: " + handshake_message.decode())

    # Reset the timeout
    arduino.timeout = timeout


def read_all(ser, read_buffer=b"", **args):
    """Read all available bytes from the serial port
    and append to the read buffer.

    Parameters
    ----------
    ser : serial.Serial() instance
        The device we are reading from.
    read_buffer : bytes, default b''
        Previous read buffer that is appended to.

    Returns
    -------
    output : bytes
        Bytes object that contains read_buffer + read.

    Notes
    -----
    .. `**args` appears, but is never used. This is for
       compatibility with `read_all_newlines()` as a
       drop-in replacement for this function.
    """
    # Set timeout to None to make sure we read all bytes
    previous_timeout = ser.timeout
    ser.timeout = None

    in_waiting = ser.in_waiting
    read = ser.read(size=in_waiting)

    # Reset to previous timeout
    ser.timeout = previous_timeout

    return read_buffer + read


def read_all_newlines(ser, read_buffer=b"", n_reads=4):
    """Read data in until encountering newlines.

    Parameters
    ----------
    ser : serial.Serial() instance
        The device we are reading from.
    n_reads : int
        The number of reads up to newlines
    read_buffer : bytes, default b''
        Previous read buffer that is appended to.

    Returns
    -------
    output : bytes
        Bytes object that contains read_buffer + read.

    Notes
    -----
    .. This is a drop-in replacement for read_all().
    """
    
    
    
    raw = read_buffer
    for _ in range(n_reads):
        raw += ser.read_until()
        
    return raw

def parse_read(read):
    """Parse a read with time, voltage data

    Parameters
    ----------
    read : byte string
        Byte string with comma delimited time/voltage
        measurements.

    Returns
    -------
    time_ms : list of ints
        Time points in milliseconds.
    voltage : list of floats
        Voltages in volts.
    remaining_bytes : byte string
        Remaining, unparsed bytes.
    """
    time_ms = []
    
    # Add more variables HERE
    # Speed
    # Brake
    # Temp
    # RPM
    # Vibration
    # Voltage
    # Current
    # Sounds
    speed = []
    brake = []
    temperature = []
    RPM_list = []
    X_accel = []
    Y_accel = []
    Z_accel = []
    Voltage = []
    Current=[]
    Sound=[]

   # Helen add more variables here


    # Separate independent time/voltage measurements
    # pattern = re.compile(b"\d+|,")
    pattern = re.compile(b"[.\d]+|,")
    # raw_list = [b"".join(pattern.findall(raw)).decode() for raw in read.split(b"\r\n")]
    
    raw_list = []
    for raw in read.split(b"\r\n"):
        patt = pattern.findall(raw)
        q = b"".join(patt)
        d = q.decode()
        raw_list.append(d)


    for raw in raw_list[:-1]:
        try:
            t, S, B, C, RPM, X, Y, Z, V, A, D = raw.split(",")
            time_ms.append(float(t))
            speed.append(float(S))
            brake.append(float(B))
            temperature.append(float(C))
            RPM_list.append(float(RPM))
            X_accel.append(float(X))
            Y_accel.append(float(Y))
            Z_accel.append(float(Z))
            Voltage.append(float(V))
            Current.append(float(A))
            Sound.append(float(D))
            # Helen add more variables here

        except:
            pass

    if len(raw_list) == 0:
        return time_ms, speed, brake, temperature, RPM_list, X_accel, Y_accel, Z_accel, Voltage, Current, Sound, b"" # Add more variables here
    else:
        return time_ms, speed, brake, temperature, RPM_list, X_accel, Y_accel, Z_accel, Voltage, Current, Sound, raw_list[-1].encode() # Add more variables here

def parse_raw(raw):
    """Parse bytes output from Arduino."""

    raw = raw.decode()
    if raw[-1] != "\n":
        raise ValueError(
            "Input must end with newline, otherwise message is incomplete."
        )

    t, Speed, brake, temperature, RPM, X, Y, Z, Voltage, Current, Sound = raw.rstrip().split(",") # Add more variables here

    return int(t), int(Speed), int(brake), int(temperature), int(RPM), int(X), int(Y), int(Z), int(Voltage), int(Current), int(Sound) # Add more variables here

def request_single_voltage(arduino):
    """Ask Arduino for a single data point"""
    # Ask Arduino for data
    arduino.write(bytes([VOLTAGE_REQUEST]))

    # Read in the data
    raw = arduino.read_until()

    # Parse and return
    return parse_raw(raw)

# map from variable name to label in the plot
label_map = {
    'Speed': 'Speed',
    'brake': 'Brake',
    'temperature' :'Temperature (C)',
    'RPM': 'RPM',
    'Vibration' : 'Vibration (G)',
    'Voltage': 'Voltage (V)',
    'Current': 'Current (A)',
    'Sound': 'Sound',
    # Add more variables here
}

def plot(mode, varname, source, y_range):
    """Build a plot of voltage vs time data"""
    y_axis_label = label_map[varname]
    # Set up plot area
    p = bokeh.plotting.figure(
        frame_width=500,
        frame_height=175,
        x_axis_label="time (s)",
        y_axis_label=y_axis_label,
        title="streaming data of " + varname if mode == "stream" else "on-demand data",
        # y_range=[-0.2, 5.2],
        y_range=y_range,
        toolbar_location="above",
        output_backend="webgl",
    )

    # No range padding on x: signal spans whole plot
    p.x_range.range_padding = 0

    # We'll sue whitesmoke backgrounds
    p.border_fill_color = "whitesmoke"

    # If we are in streaming mode, use a line, dots for on-demand
    if mode == "stream":
        p.line(source=source, x="t", y=varname)
    else:
        p.circle(source=source, x="t", y=varname)

    # Put a phantom circle so axis labels show before data arrive
    phantom_source = bokeh.models.ColumnDataSource(data=dict(t=[0], 
                                                             Speed=[0], brake=[0], temperature=[0], 
                                                             RPM=[0], 
                                                             Vibration=[0], 
                                                             Voltage=[0],
                                                             Current=[0],
                                                             Sound=[0],
                                                             )) # Add more variables here

    # Commented out because it was causing the topmost plot to always have x = 0 in the plot 
    # even when the current data window was not in that range. 
    # The downside to commenting this out is, because as the comment says, no axes are shown
    # before plotting and also after doing a Reset.
    # TODO Should someday fix so it does what it was meant to do without the negative side effect
    # p.circle(source=phantom_source, x="t", y=varname, visible=False)

    return p, source, phantom_source


def controls(mode):
    if mode == "stream":
        acquire = bokeh.models.Toggle(label="stream", button_type="success", width=100)
        save_notice = bokeh.models.Div(
            text="<p>No streaming data saved.</p>", width=165
        )
    else:
        acquire = bokeh.models.Button(label="acquire", button_type="success", width=100)
        save_notice = bokeh.models.Div(
            text="<p>No on-demand data saved.</p>", width=165
        )

    save = bokeh.models.Button(label="save", button_type="primary", width=100)
    reset = bokeh.models.Button(label="reset", button_type="warning", width=100)
    file_input = bokeh.models.TextInput(
        title="file name", value=f"{mode}.csv", width=165
    )

    return dict(
        acquire=acquire,
        reset=reset,
        save=save,
        file_input=file_input,
        save_notice=save_notice,
    )


def layout(p, ctrls):
    buttons = bokeh.layouts.row(
        bokeh.models.Spacer(width=30),
        ctrls["acquire"],
        bokeh.models.Spacer(width=295),
        ctrls["reset"],
    )
    left = bokeh.layouts.column(p, buttons, spacing=15)
    right = bokeh.layouts.column(
        bokeh.models.Spacer(height=50),
        ctrls["file_input"],
        ctrls["save"],
        ctrls["save_notice"],
    )
    return bokeh.layouts.row(
        left, right, spacing=30, margin=(30, 30, 30, 30), background="whitesmoke",
    )


def acquire_callback(arduino, stream_data, source, phantom_source, rollover):
    # does not get called when streaming

    # Pull t and V values from stream or request from Arduino
    if stream_data["mode"] == "stream":
        t = stream_data["t"][-1]
        V = stream_data["V"][-1]
        brake = stream_data["brake"][-1]
    else:
        t, V, brake = request_single_voltage(arduino)

    # Add to on-demand data dictionary
    on_demand_data["t"].append(t)
    on_demand_data["V"].append(V)
    on_demand_data["brake"].append(brake)

    # Send new data to plot
    new_data = dict(t=[t / 1000], V=[V], brake=[brake])
    source.stream(new_data, rollover=rollover)

    # Update the phantom source to keep the x_range on plot ok
    phantom_source.data = new_data


def stream_callback(arduino, stream_data, new):
    if new:
        stream_data["mode"] = "stream"
    else:
        stream_data["mode"] = "on-demand"
        arduino.write(bytes([ON_REQUEST]))

    arduino.reset_input_buffer()


def reset_callback(mode, data, source, phantom_source, controls):
    # Turn off the stream
    if mode == "stream":
        controls["acquire"].active = False

    # Black out the data dictionaries
    data["t"] = []
    data["Speed"] = []
    data["brake"] = []
    data["temperature"] = []
    data["RPM"] = []
    data["X"] = []
    data["Y"] = []
    data["Z"] = []
    data["Voltage"] = []
    data["Current"] = []
    data["Sound"] = []
    # Add more variables here

    # Reset the sources
    source.data = dict(t=[], Speed=[], brake=[], temperature=[], 
                       RPM=[], 
                       Vibration=[], 
                       Voltaqe=[], 
                       Current=[], Sound=[])  # Add more variables here
    phantom_source.data = dict(t=[0], Speed=[0], brake=[0], temperature=[0], 
                               RPM=[0], 
                               Vibration=[0], 
                               Voltage=[0], 
                               Current=[0], Sound=[0])  # Add more variables here


def save_callback(mode, data, controls):
    # Convert data to data frame and save
    df = pd.DataFrame(data={"time (ms)": data["t"], "Speed": data["Speed"], 
                            "Brake": data["brake"],
                            "temperature": data["temperature"],
                            "RPM": data["RPM"],
                            "Vibration": data["Vibration"],
                            "Voltage": data["Voltage"],
                            "Current": data["Current"],
                            "Sound": data["Sound"],
                            # Add more variables here
                            })
    df.to_csv(controls["file_input"].value, index=False)

    # Update notice text
    notice_text = "<p>" + ("Streaming" if mode == "stream" else "On-demand")
    notice_text += f" data was last saved to {controls['file_input'].value}.</p>"
    controls["save_notice"].text = notice_text


def disable_controls(controls):
    """Disable all controls."""
    for key in controls:
        controls[key].disabled = True



def stream_update(data, source, phantom_source, rollover):
    # Update plot by streaming in data

    new_data = {
        "t": list(np.array(data["t"][data["prev_array_length"] :]) / 1000),
        "Speed": data["Speed"][data["prev_array_length"] :],
        "brake": data["brake"][data["prev_array_length"] :],
        "temperature": data["temperature"][data["prev_array_length"] :],
        "RPM": data["RPM"][data["prev_array_length"] :],
        "Vibration": data["Vibration"][data["prev_array_length"] :],
        "Voltage": data["Voltage"][data["prev_array_length"] :],
        "Current": data["Current"][data["prev_array_length"] :],
        "Sound": data["Sound"][data["prev_array_length"] :],
        # Add more variables here
    }
    
    source.stream(new_data, rollover)

    # Adjust new phantom data point if new data arrived
    if len(new_data["t"]) > 0:
        phantom_source.data = dict(t=[new_data["t"][-1]], Speed=[new_data["Speed"][-1]], 
                                   brake=[new_data["brake"][-1]],
                                   temperature=[new_data["temperature"][-1]],
                                   RPM=[new_data["RPM"][-1]],
                                   Vibration=[new_data["Vibration"][-1]],
                                   Voltage=[new_data["Voltage"][-1]],
                                   Current=[new_data["Current"][-1]],
                                   Sound=[new_data["Sound"][-1]],
                                   # Add more variables here
                                   )
    data["prev_array_length"] = len(data["t"])


def potentiometer_app(
    arduino, stream_data, on_demand_data, daq_task, rollover=400, stream_plot_delay=90,
):
    def _app(doc):
        # Plots
        stream_source = bokeh.models.ColumnDataSource(data=dict(t=[], Speed=[], brake=[], 
                                                                temperature=[], 
                                                                RPM=[],
                                                                Vibration=[],
                                                                Voltage=[],
                                                                Current=[],
                                                                Sound=[],
                                                                ))
        on_demand_source = bokeh.models.ColumnDataSource(data=dict(t=[], Speed=[], brake=[], 
                                                                   temperature=[], 
                                                                   RPM=[],
                                                                   Vibration=[],
                                                                   Voltage=[],
                                                                   Current=[],
                                                                   Sound=[],
                                                                   ))

        p_stream_S, stream_source, stream_phantom_source = plot("stream", "Speed", stream_source, [-0.2, 260.])
        p_stream_brake, stream_source, stream_phantom_source = plot("stream", "brake", stream_source, [-0.2, 8.])
        p_stream_temperature, stream_source, stream_phantom_source = plot("stream", "temperature", stream_source, [20., 30.])
        p_stream_RPM, stream_source, stream_phantom_source = plot("stream", "RPM", stream_source, [-0.2, 250.])
        p_stream_Vibration, stream_source, stream_phantom_source = plot("stream", "Vibration", stream_source, [-0.2, 3.])
        p_stream_Voltage, stream_source, stream_phantom_source = plot("stream", "Voltage", stream_source, [-0.2, 8.])
        p_stream_Current, stream_source, stream_phantom_source = plot("stream", "Current", stream_source, [-0.2, 1200.])
        p_stream_Sound, stream_source, stream_phantom_source = plot("stream", "Sound", stream_source, [-0.2, 300.])
        # Add more variables here
        p_on_demand, on_demand_source, on_demand_phantom_source = plot("on demand", "Speed", on_demand_source, [-0.2, 120.])

        # Controls
        stream_controls = controls("stream")
        on_demand_controls = controls("on_demand")

        # Shut down
        shutdown_button = bokeh.models.Button(
            label="shut down", button_type="danger", width=100
        )
        
        # brake_input = TextInput(value="0", title="Brake Value:")

        # brake_set_button = bokeh.models.Button(
        #     label="SET BRAKE", button_type="success", width=100
        # )

        # speed_input = TextInput(value="0", title="Speed Value:")

        # speed_set_button = bokeh.models.Button(
        #     label="SET SPEED", button_type="success", width=100
        # )

        brake_reset_button = bokeh.models.Button(
            label="RESET", button_type="danger", width=100
        )

        speed_reset_button = bokeh.models.Button(
            label="RESET", button_type="danger", width=100
        )
        
        slider_speed = Slider(start=0, end=255, value=0, step=15, title="Speed")

        slider_brake = Slider(start=0, end=6, value=0, step=1, title="Brake")

        slider_speed_ramp = Slider(start=0, end=5, value=0, step=.5, title="Ramping for Speed")

        ramp_speed_reset_button = bokeh.models.Button(
            label="RESET", button_type="danger", width=100
        )

        slider_brake_ramp = Slider(start=0, end=0.5, value=0, step=0.05, title="Ramping for Break")

        ramp_brake_reset_button = bokeh.models.Button(
            label="RESET", button_type="danger", width=100
        )

        heat_button = bokeh.models.Button(
            label="HEAT ON", button_type="success", width=100
        )

        heat_OFF_button = bokeh.models.Button(
            label="HEAT OFF", button_type="danger", width=100
        )

        # Layouts
        stream_S_layout = layout(p_stream_S, stream_controls)
        stream_brake_layout = layout(p_stream_brake, stream_controls)
        stream_temperature_layout = layout(p_stream_temperature, stream_controls)
        stream_RPM_layout = layout(p_stream_RPM, stream_controls)
        stream_Vibration_layout = layout(p_stream_Vibration, stream_controls)
        stream_Voltage_layout = layout(p_stream_Voltage, stream_controls)
        stream_Current_layout = layout(p_stream_Current, stream_controls)
        stream_Sound_layout = layout(p_stream_Sound, stream_controls)
        # Add more variables here
        on_demand_layout = layout(p_on_demand, on_demand_controls)

        # Shut down layout
        shutdown_layout = bokeh.layouts.row(
            # bokeh.models.Spacer(width=675), shutdown_button, led_on_button, led_off_button
            slider_speed, speed_reset_button, slider_brake, brake_reset_button, shutdown_button
        )

        ramp_layout = bokeh.layouts.row(
            # bokeh.models.Spacer(width=675), shutdown_button, led_on_button, led_off_button
            slider_speed_ramp, ramp_speed_reset_button, slider_brake_ramp, ramp_brake_reset_button
        )

        heat_layout = bokeh.layouts.row(
            # bokeh.models.Spacer(width=675), shutdown_button, led_on_button, led_off_button
            heat_button, heat_OFF_button
        )

        app_layout = bokeh.layouts.gridplot(
            [[shutdown_layout], [ramp_layout], [heat_layout], [stream_S_layout, stream_brake_layout], 
            [stream_temperature_layout, 
            stream_RPM_layout], 
            [stream_Vibration_layout, 
            stream_Voltage_layout], 
            [stream_Current_layout, 
            stream_Sound_layout], 
            [on_demand_layout]] # Add more variables here
            # shutdown_layout, stream_V_layout, on_demand_layout
        )

        def _acquire_callback(event=None):
            acquire_callback(
                arduino,
                stream_data,
                on_demand_source,
                on_demand_phantom_source,
                rollover,
            )

        def _stream_callback(attr, old, new):
            stream_callback(arduino, stream_data, new)

        def _stream_reset_callback(event=None):
            reset_callback(
                "stream",
                stream_data,
                stream_source,
                stream_phantom_source,
                stream_controls,
            )

        def _on_demand_reset_callback(event=None):
            reset_callback(
                "on demand",
                on_demand_data,
                on_demand_source,
                on_demand_phantom_source,
                on_demand_controls,
            )

        def _stream_save_callback(event=None):
            save_callback("stream", stream_data, stream_controls)

        def _on_demand_save_callback(event=None):
            save_callback("on demand", on_demand_data, on_demand_controls)

        def _shutdown_callback(event=None):
            print("ok it has to be here")
            # shutdown_callback(
            #     arduino, daq_task, stream_data, stream_controls, on_demand_controls
            # )

        # def _brake_set_callback(event=None):
        #     # arduino.write(bytes("BRAKE " + brake_input.value, "utf-8"))
        #     data = " BRAKE " + brake_input.value
        #     arduino.write(data.encode())
        #     response = arduino.readline().decode().strip()  # Read and decode data from Arduino
        #     print("Arduino says:", response)

        # def _speed_set_callback(event=None):
        #     # arduino.write(bytes("PWM " + speed_input.value, "utf-8"))
        #     data = " PWM " + speed_input.value
        #     arduino.write(data.encode())
        #     response = arduino.readline().decode().strip()  # Read and decode data from Arduino
        #     print("Arduino says:", response)

        def brake_reset_callback(event=None):
            dataBrake = " BRAKE 0"
            slider_brake.value = 0
            arduino.write(dataBrake.encode())
        
        def speed_reset_callback(event=None):
            dataSpeed = " PWM 0"
            slider_speed.value = 0
            arduino.write(dataSpeed.encode())
        
        def slider_speed_callback(attr, old, new):
            slider_speed.title = "Speed"
            data = " PWM " + str(slider_speed.value)
            arduino.write(data.encode())

        def slider_brake_callback(attr, old, new):
            slider_brake.title = "Brake"
            data = " BRAKE " + str(slider_brake.value)
            arduino.write(data.encode())

        def slider_speed_ramp_callback(attr, old, new):
            slider_speed_ramp.title = "Ramping for Speed"
            slider_speed.title = "invalid - slide again to use"
            data = " SPEEDRAMP " + str(slider_speed_ramp.value)
            arduino.write(data.encode())

        def slider_brake_ramp_callback(attr, old, new):
            slider_brake_ramp.title = "Ramping for Brake"
            slider_brake.title = "invalid -  slide again to use"
            data = " BRAKERAMP " + str(slider_brake_ramp.value)
            arduino.write(data.encode())
        
        def ramp_speed_reset_callback(event=None):
            data = " SPEEDRAMP 0"
            slider_speed_ramp.value = 0
            arduino.write(data.encode())
        
        def ramp_brake_reset_callback(event=None):
            data = " BRAKERAMP 0"
            slider_brake_ramp.value = 0
            arduino.write(data.encode())

        def heat_callback(event=None):
            data = " HEAT 255"
            arduino.write(data.encode())

        def heat_OFF_callback(event=None):
            data = " HEAT 0"
            arduino.write(data.encode())

        def shutdown_callback(
            arduino, daq_task, stream_data, stream_controls, on_demand_controls
        ):

            print("here???")
            # Disable controls
            disable_controls(stream_controls)
            disable_controls(on_demand_controls)

            # Strop streaming
            stream_data["mode"] = "on-demand"
            arduino.write(bytes([ON_REQUEST]))

            print("did something happen")

            speed_reset_callback()
            brake_reset_callback()
        

            # Stop DAQ async task
            daq_task.cancel()

            # Disconnect from Arduino
            arduino.close()

        @bokeh.driving.linear()
        def _stream_update(step):
            stream_update(stream_data, stream_source, stream_phantom_source, rollover)

            # Shut down server if Arduino disconnects (commented out in Jupyter notebook)
            if not arduino.is_open:
                sys.exit()

        # Link callbacks
        stream_controls["acquire"].on_change("active", _stream_callback)
        stream_controls["reset"].on_click(_stream_reset_callback)
        stream_controls["save"].on_click(_stream_save_callback)
        on_demand_controls["acquire"].on_click(_acquire_callback)
        on_demand_controls["reset"].on_click(_on_demand_reset_callback)
        on_demand_controls["save"].on_click(_on_demand_save_callback)
        shutdown_button.on_click(_shutdown_callback)
        # brake_set_button.on_click(_brake_set_callback)
        # speed_set_button.on_click(_speed_set_callback)
        brake_reset_button.on_click(brake_reset_callback)
        speed_reset_button.on_click(speed_reset_callback)
        slider_speed.on_change('value_throttled', slider_speed_callback)
        slider_brake.on_change('value_throttled', slider_brake_callback)
        slider_speed_ramp.on_change('value_throttled', slider_speed_ramp_callback)
        slider_brake_ramp.on_change('value_throttled', slider_brake_ramp_callback)
        ramp_speed_reset_button.on_click(ramp_speed_reset_callback)
        ramp_brake_reset_button.on_click(ramp_brake_reset_callback)
        heat_button.on_click(heat_callback)
        heat_OFF_button.on_click(heat_OFF_callback)

        # Add the layout to the app
        doc.add_root(app_layout)

        # Add a periodic callback, monitor changes in stream data
        pc = doc.add_periodic_callback(_stream_update, stream_plot_delay)

    return _app


# Set up connection
HANDSHAKE = 0
VOLTAGE_REQUEST = 1
ON_REQUEST = 2
STREAM = 3
READ_DAQ_DELAY = 4

port = find_arduino()

arduino = serial.Serial(port, baudrate=38400)
handshake_arduino(arduino)

# Set up data dictionaries
# Add more variables here
stream_data = dict(prev_array_length=0, t=[], Speed=[], brake=[], temperature=[], 
                   RPM=[], 
                   Vibration=[], 
                   Voltage=[], 
                   Current=[], 
                   Sound=[], 
                   mode="on demand") # Add more variables here
on_demand_data = dict(t=[], Speed=[], brake=[], temperature=[], 
                      RPM=[],
                      Vibration=[],
                      Voltage=[],
                      Current=[],
                      Sound=[],
                      ) # Add more variables here


async def daq_stream_async(
    arduino,
    data,
    delay=20,
    n_trash_reads=5,
    n_reads_per_chunk=4,
    reader=read_all_newlines,
):
    
    """Obtain streaming data"""
    # Specify delay
    arduino.write(bytes([READ_DAQ_DELAY]) + (str(delay) + "x").encode())

    # Current streaming state
    stream_on = False

    # Receive data
    read_buffer = [b""]
    while True:
        if data["mode"] == "stream":
            # Turn on the stream if need be
            if not stream_on:
                arduino.write(bytes([STREAM]))

                # Read and throw out first few reads
                i = 0
                while i < n_trash_reads:
                    _ = arduino.read_until()
                    i += 1

                stream_on = True

            # Read in chunk of data
            raw = reader(arduino, read_buffer=read_buffer[0], n_reads=n_reads_per_chunk)
            
            
            # Parse it, passing if it is gibberish
            try:
                # Add more variables here
                t, Speed, brake, temperature, RPM, X, Y, Z, Voltage, Current, Sound, read_buffer[0] = parse_read(raw) # Add more variables here
                
                # Update data dictionary
                data["t"] += t
                data["Speed"] += Speed
                data["brake"] += brake 
                data["temperature"] += temperature
                data["RPM"] += RPM
                
                Vibration = [math.sqrt(x**2 + y**2 + z**2) for x, y, z in zip(X, Y, Z)]

                data["Vibration"] += Vibration
                data["Voltage"] += Voltage
                data["Current"] += Current
                data["Sound"] += Sound
                # Add more variables here
            except Exception as e:
                pass
        else:
            # Make sure stream is off
            stream_on = False

        # Sleep 80% of the time before we need to start reading chunks
        await asyncio.sleep(0.8 * n_reads_per_chunk * delay / 1000)


# Set up asynchronous DAQ task
daq_task = asyncio.create_task(daq_stream_async(arduino, stream_data))

# Build app
app = potentiometer_app(
    arduino, stream_data, on_demand_data, daq_task, rollover=100, stream_plot_delay=90
)

# Build it with curdoc
app(bokeh.plotting.curdoc())