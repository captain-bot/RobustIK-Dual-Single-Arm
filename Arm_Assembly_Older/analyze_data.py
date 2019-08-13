import pickle

dbfile = open("dual_arm_assembly", "rb")
db = pickle.load(dbfile)
data = None
for key in db:
    print(key)
    if key == "assembly_err_list":
        data = db[key]
        data.sort()
dbfile.close()

print(data[-10500:-10450])
