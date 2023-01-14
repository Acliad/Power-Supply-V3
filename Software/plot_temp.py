#%%
import serial
import time
import serial.tools.list_ports
import matplotlib.pyplot as plt
from IPython import display

%matplotlib inline

#%%
ports = serial.tools.list_ports.comports()
for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))

# %%
teensy_port = '/dev/cu.usbmodem15708201'
teensy = serial.Serial(port=teensy_port, timeout=1)

def getTemp():
    teensy.reset_input_buffer()
    teensy.write(bytes("temp", 'utf-8'))
    time.sleep(0.1)
    response = teensy.readline()

    return float(response.strip())

#%%


print(getTemp())
# %%
TEST_TIME_S = 8 * 60
SAMPLE_PERIOD_S = 1

temps = []
times = []

# plt.close()
# plt.plot(times, temps)
# ax = plt.gca()

teensy.write(bytes("pid on\n", 'utf-8'))
time.sleep(0.1)
start_time = time.time()
last_update_time = start_time
while (time.time() - start_time < TEST_TIME_S):
    last_update_time = time.time()
    temps.append(getTemp())
    times.append(time.time() - start_time)
    # plt.clf()
    plt.plot(times, temps)
    # display.display(plt.gcf())
    plt.show()
    display.clear_output(wait=True)
    time.sleep(SAMPLE_PERIOD_S)

# %%

plt.plot(times, temps)
plt.xlabel('Time (sec)')
plt.ylabel('Temp (°C)')
plt.grid()
# %%
