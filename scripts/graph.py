import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

data = json.load(open('data.json'))['data']

time = map(lambda x: x['time'], data)
area = map(lambda x: x['area'], data)
cost = map(lambda x: x['cost']*3000, data)

fig = plt.figure()


plt.plot(time, cost, 'b--', label='cost')
plt.plot(time, area, 'r', label='area')
red_patch = mpatches.Patch(color='red', label='The red data')
plt.legend()
fig.suptitle('Mapped area', fontsize=14, fontweight='bold')

ax = fig.add_subplot(111)
fig.subplots_adjust(top=0.85)
ax.set_title('axes title')

ax.set_xlabel('time (seconds)')
ax.set_ylabel('area (mapped)')

plt.savefig('data.png');
plt.clf()

