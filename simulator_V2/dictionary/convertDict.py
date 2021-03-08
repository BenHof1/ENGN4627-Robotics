import csv

f = open("aruco4x4.txt",'r')
wf = open("aruco4x4.csv", 'w')
wc = csv.writer(wf)

counter = 0
for row in f:
    row = row.strip()
    for i in range(5):
        row = row.rstrip(';')
        row = row.lstrip('{')
        row = row.lstrip(' ')
        row = row.rstrip('}')
        row = row.rstrip(' ')
        row = row.rstrip(',')
        
    row = row.split(',')
    row = [int(r) for r in row]

    wc.writerow(row)
    

    # print(row)
    counter += 1


f.close()
wf.close()