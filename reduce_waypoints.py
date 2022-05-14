with open("new_waypoints2.csv", "r") as inp:
    wpts = inp.readlines() 
    inp.close()
    with open("less_waypoints_new2.csv", "w+") as out:
        for i in range(len(wpts)):
            if i % 10 == 0:
                out.write(wpts[i])
        out.close()