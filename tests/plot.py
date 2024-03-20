import numpy as np
import matplotlib.pyplot as plt

tasks = ['Needle Reach', 'Gauze Retrieve', 'Peg Transfer']
groups = ['Free Train', 'Haptic Guidance Train']
indicators = ['Average Complete Time (s)', 'Average Trajectory Length', 'Success Rate (%)', 'Average Distance to Target']

record_data = {}
record_data['self_needle'] = [[3.85,3.65,3.35,5.70,4.90,6.48,10.80,4.21,7.56,8.56], # Complete Time (s)
                              [2.682,2.919,3.074,4.176,4.694,3.107,5.351,2.735,5.442,4.692], # Trajectory Length
                              [3    ,1    ,0    ,2    ,0    ], # Drop Times
                              [0    ,0    ,0    ,0    ,0    ], # Distance to Target
                              ]
record_data['demo_needle'] = [[3.03,2.92,2.51,4.11,2.98,3.98,4.35,3.20,3.74,3.00], # Complete Time (s)
                              [3.344,3.013,3.259,4.673,2.949,3.577,3.373,3.262,3.810,3.078], # Trajectory Length
                              [3    ,1    ,0    ,2    ,0    ], # Drop Times
                              [0    ,0    ,0    ,0    ,0    ], # Distance to Target
                              ]

record_data['self_gauze']  = [[10.41,6.60,6.91,15.79,9.18,8.45,13.41,12.12,18.90,12.18], # Complete Time (s)
                              [5.409,3.705,3.789,8.517,3.502,3.575,6.592,7.516,6.844,7.743], # Trajectory Length
                              [3    ,1    ,0    ,2    ,0    ], # Drop Times
                              [0    ,0    ,0    ,0    ,0    ], # Distance to Target
                              ]
record_data['demo_gauze']  = [[8.26,6.86,5.93,7.18,8.20,8.33,5.42,7.22,5.54,6.27], # Complete Time (s)
                              [8.259,7.256,3.573,6.316,6.885,9.249,3.558,8.025,3.892], # Trajectory Length
                              [3    ,1    ,0    ,2    ,0    ], # Drop Times
                              [0    ,0    ,0    ,0    ,0    ], # Distance to Target
                              ]
record_data['self_peg']    = [[44.02,25.23,15.30,45.34,17.98], # Complete Time (s)
                              [24.92,15.03,10.79,29.19,12.39], # Trajectory Length
                              [3    ,1    ,0    ,2    ,0    ], # Drop Times
                              [0    ,0    ,0    ,0    ,0    ], # Distance to Target
                              ]
record_data['demo_peg']    = [[16.79,26.77,13.38,10.78,12.65], # Complete Time (s)
                              [6.68 ,19.96,5.45 ,6.1  ,10.36], # Trajectory Length
                              [0    ,2    ,0    ,0    ,0    ], # Drop Times
                              [0    ,0    ,0    ,0    ,0    ], # Distance to Target
                              ]
record_error={}
for data in record_data:
    # print(max(record_data[data][0]))
    # exit()
    time_error = [max(record_data[data][0]) , min(record_data[data][0])]
    dis_error = [max(record_data[data][1]) , min(record_data[data][1])]
    rate_error = [max(record_data[data][2]) , min(record_data[data][2])]
    targe_error = [max(record_data[data][3]) , min(record_data[data][3])]
    # print(time_error)
    name = str(data)
    record_error[name] = [time_error, dis_error, rate_error, targe_error]
# print(record_error)
# tasks × groups × indicators
data = np.array([[[np.mean(record_data['self_needle'][0]), np.mean(record_data['self_needle'][1]), np.mean(record_data['self_needle'][2]), np.mean(record_data['self_needle'][3])],  # Needle Reach - Free Train
                  [np.mean(record_data['demo_needle'][0]), np.mean(record_data['demo_needle'][1]), np.mean(record_data['demo_needle'][2]), np.mean(record_data['demo_needle'][3])]], # Needle Reach - Haptic Guidance Train
                 [[np.mean(record_data['self_gauze'][0]), np.mean(record_data['self_gauze'][1]), np.mean(record_data['self_gauze'][2]), np.mean(record_data['self_gauze'][3])],  # Gauze Retrieve - Free Train
                  [np.mean(record_data['demo_gauze'][0]), np.mean(record_data['demo_gauze'][1]), np.mean(record_data['demo_gauze'][2]), np.mean(record_data['demo_gauze'][3])]], # Gauze Retrieve - Haptic Guidance Train
                 [[np.mean(record_data['self_peg'][0]), np.mean(record_data['self_peg'][1]), np.mean(record_data['self_peg'][2]), np.mean(record_data['self_peg'][3])],  # Peg Transfer - Free Train
                  [np.mean(record_data['demo_peg'][0]), np.mean(record_data['demo_peg'][1]), np.mean(record_data['demo_peg'][2]), np.mean(record_data['demo_peg'][3])]]])# Peg Transfer - Haptic Guidance Train

fig, axs = plt.subplots(2, 2, figsize=(12, 8))

# for i in range(len(indicators)):
#     ax = axs[i//2, i%2]
#     for j, group in enumerate(groups):

#         print('dddddddddddddd',data[:, j, i])
#         ax.bar(np.arange(len(tasks)) + 0.4 * j, data[:, j, i], width=0.4, label=group) 
#         ax.errorbar()
#     ax.set_xticks(np.arange(len(tasks)))
#     ax.set_xticklabels(tasks)
#     ax.set_ylabel(indicators[i])
#     ax.set_title(indicators[i])
#     ax.legend()

# plt.tight_layout()
# plt.show()

for i in range(len(indicators)):
    ax = axs[i//2, i%2]
    for j, group in enumerate(groups):
        means = data[:, j, i]
        errors = [record_error[data][i] for data in record_data]
        print('ssssssssssssssss',means)
        print('aaaaaaaaaaaaaaa',errors)
        # errors = [record_error[group][i] for group in groups]
        ax.bar(np.arange(len(tasks)) + 0.4 * j, means, width=0.4, label=group) 
        center = [(x[0] + x[1]) / 2 for x in errors]
        error = [(x[0] - x[1]) / 2 for x in errors]
        
        for idx, (x, y, yerr) in enumerate(zip(np.arange(len(tasks)) + 0.4 * j, [center[j],center[j+2],center[j+4]], [error[j],error[j+2],error[j+4]])):
            ax.errorbar(x, y, yerr=yerr, fmt='none', ecolor='k', capsize=5)
    ax.set_xticks(np.arange(len(tasks)))
    ax.set_xticklabels(tasks)
    ax.set_ylabel(indicators[i])
    ax.set_title(indicators[i])
    ax.legend()

plt.tight_layout()
plt.show()
