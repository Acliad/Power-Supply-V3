#%%
import scipy.io as spio
from scipy.optimize import curve_fit as spfit
import numpy as np
import matplotlib.pyplot as plt

#%%
ADC_NUM_BITS = 1
def fit_func(x, A, B):
    return A*np.exp(B*x)
 
mat_data = spio.loadmat('tempDataPoints.mat')
temp_data = mat_data['temperatureData']
adc_raw = temp_data[:, 0]
adc_raw_normalized = temp_data[:, 0] / (2**ADC_NUM_BITS)
temp_c = temp_data[:, 1]

# %%
plt.plot(temp_data[:, 0], temp_data[:, 1])
plt.grid()
# %%
plt.figure()
plt.plot(temp_data[:, 0])
plt.figure()
plt.plot(temp_data[:, 1])
plt.figure()
plt.plot(temp_data[:, 2])
# %%
init_guess = (120, -1)
popt, pcov = spfit(fit_func, adc_raw_normalized, temp_c, p0=init_guess)

A = popt[0]
B = popt[1]

print(f"A = {A}")
print(f"B = {B}")
print(f"Covariance = {pcov}")

plt.plot(adc_raw, temp_c)
plt.plot(adc_raw, fit_func(adc_raw_normalized, A, B))
plt.legend(('Measured', 'Mapped'))
plt.grid()

# %%
