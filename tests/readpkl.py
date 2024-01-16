import pickle 

file = open('/home/kj/skjsurrol/SurRoL_skj/tests/save_file.pkl','rb')
data = pickle.load(file)
file.close()
print('showing:')
idx = 0
for item in data:
    print('self.path',idx,'is:',item)
    idx += 1