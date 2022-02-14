import csv
import matplotlib.pyplot as plt

f = open('results_m7.csv', 'r')

with f:

    reader = csv.reader(f)
    index = 0
    for row in reader:
        if index==0:
            m = row[1]

        elif index==1:
            output_number_of_piconets = [float(i) for i in row[1:]]

        elif index==2:
            output_ble_default_collision = [float(i) for i in row[1:]]

        elif index==3:
            output_remove_bad_channels_collision = [float(i) for i in row[1:]]

        elif index==4:
            output_experimental_collision = [float(i) for i in row[1:]]

        elif index==5:
            avg_bw_default = [float(i) for i in row[1:]]

        elif index==6:
            avg_bw_sim = [float(i) for i in row[1:]]

        elif index==7:
            avg_bw_ex = [float(i) for i in row[1:]]

        elif index==8:
            avg_predict_bw_default = [float(i) for i in row[1:]]

        elif index==9:
            avg_predict_bw_sim = [float(i) for i in row[1:]]

        elif index==10:
            avg_predict_bw_ex = [float(i) for i in row[1:]]

        index+=1


fig, ax = plt.subplots()
ax.plot(output_number_of_piconets, output_ble_default_collision, 'k', label='BLE')
ax.plot(output_number_of_piconets, output_remove_bad_channels_collision, 'k--', label='CR')
ax.plot(output_number_of_piconets, output_experimental_collision, 'k:', label='CS')
for label in (ax.get_xticklabels() + ax.get_yticklabels()):
	label.set_fontsize(12)
plt.ylabel('Collision Probability',fontsize=12)
plt.xlabel('Number of Piconets',fontsize=12)
plt.legend(['BLE', 'CR', 'CS'], loc='upper left',fontsize=12)
plt.ylim([0, 1])
plt.show()

fig, ax = plt.subplots()
ax.plot(output_number_of_piconets, avg_predict_bw_default, 'k', label='BLE')
ax.plot(output_number_of_piconets, avg_predict_bw_sim, 'k--', label='CR')
ax.plot(output_number_of_piconets, avg_predict_bw_ex, 'k:', label='CS')
for label in (ax.get_xticklabels() + ax.get_yticklabels()):
	label.set_fontsize(12)
plt.ylabel('Bandwidth prediction',fontsize=12)
plt.xlabel('Number of Piconets',fontsize=12)
plt.legend(['BLE', 'CR', 'CS'], loc='upper left',fontsize=12)
plt.ylim([0, 1.1])
plt.show()

fig, ax = plt.subplots()
ax.plot(output_number_of_piconets, avg_bw_default, 'k', label='BLE')
ax.plot(output_number_of_piconets, avg_bw_sim, 'k--', label='CR')
ax.plot(output_number_of_piconets, avg_bw_ex, 'k:', label='CS')
for label in (ax.get_xticklabels() + ax.get_yticklabels()):
	label.set_fontsize(12)
plt.ylabel('Bandwidth Indicator',fontsize=12)
plt.xlabel('Number of Piconets',fontsize=12)
plt.legend(['BLE', 'CR', 'CS'], loc='upper left',fontsize=12)
plt.ylim([0, 1.1])
plt.show()