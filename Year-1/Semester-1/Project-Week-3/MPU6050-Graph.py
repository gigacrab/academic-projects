import pandas as pd
import matplotlib.pyplot as plt


# 1. Read the CSV file into a pandas DataFrame
# Replace 'data.csv' with the path to your file
df = pd.read_csv('MPU6050-Data.csv')

# 2. Extract columns for plotting
# The 'Month' column for the x-axis, 'Sales' for the y-axis
time_data = df['Time']
pitch_data = df['Pitch']
filtered_pitch_data = df['Filtered Pitch']

# 3. Create the plot
plt.figure(figsize=(8, 5)) # Optional: Adjust figure size
plt.plot(time_data, pitch_data, 'r-', label='Unfiltered pitch')
plt.plot(time_data, filtered_pitch_data, 'b-', label='Kalman-filtered pitch')

# 4. Add labels and a title
plt.xlabel('Time(ms)')
plt.ylabel('Pitch(°)')
plt.title('Unfiltered pitch and Kalman-filtered pitch versus time')
plt.legend()
plt.grid()

# 5. Display the graph
plt.xticks(rotation=45) # Optional: Rotate x-axis labels for better readability
plt.tight_layout() # Optional: Adjust layout to prevent labels from being cut off
plt.show()