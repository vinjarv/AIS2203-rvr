import datetime as dt
import gantt as gt

gt.define_font_attributes(fill='black', stroke='black', stroke_width=0, font_family="Verdana")

rHLR = gt.Resource('HLR')
rVV = gt.Resource('VV')
rMMM = gt.Resource('MMM')

tasks = []
def makeTask(sTag, iDay, iMonth, iNumDays, iPercent, aOwners):
    tasks.append(gt.Task(name=sTag,
                         start=dt.date(2022, iMonth, iDay),
                         duration=iNumDays,
                         percent_done=iPercent,
                         resources=aOwners,
                         color='violet'))

def makeSubtask(sTag, iDay, iMonth, iNumDays, iPercent, aOwners):
    tasks.append(gt.Task(name=sTag,
                         start=dt.date(2022, iMonth, iDay),
                         duration=iNumDays,
                         percent_done=iPercent,
                         resources=aOwners,
                         color='lightgreen'))

def makeMilestone(sTag, aDepends):
    tasks.append(gt.Milestone(name=sTag,
                              depends_of=aDepends))

def makeProject(sTag, aTasks):
    project = gt.Project(name=sTag)
    for i in range(len(aTasks)):
        project.add_task(aTasks[i])
    return project

#MAIN #8 (0-7)
makeTask('MAIN PROGRAM (C++)', 17, 10, 30, 5, [rHLR, rVV, rMMM])
makeSubtask('Structure', 17, 10, 18, 15, [rVV])
makeSubtask('Manual control (arduino)', 31, 10, 10, 20, [rVV, rMMM])
makeSubtask('Input handling', 2, 11, 10, 0, [rVV])
makeSubtask('Crash control', 7, 11, 8, 0, [rVV])
makeSubtask('Object recognition (openCV)', 7, 11, 13, 0, [rVV])
makeSubtask('Movement', 9, 11, 11, 0, [rVV])
makeSubtask('Lifting and placing', 14, 11, 10, 0, [rVV])


#GUI #6 (8-13)
makeTask('GUI (C++, Qt)', 17, 10, 30, 75, [rMMM])
makeSubtask('Test values', 17, 10, 8, 0, [rMMM])
makeSubtask('Layout', 24, 10, 5, 0, [rMMM])
makeSubtask('Video display', 25, 10, 9, 0, [rMMM])
makeSubtask('Buttons and meters', 31, 10, 10, 0, [rMMM])
makeSubtask('Error handling', 7, 11, 10, 0, [rMMM])


#rPI #5 (14-18)
makeTask('RASPBERRY PI (python)', 17, 10, 30, 20, [rHLR])
makeSubtask('Contact and test values', 17, 10, 15, 0, [rHLR])
makeSubtask('Input handling', 31, 10, 8, 0, [rHLR])
makeSubtask('Video transmission', 2, 11, 8, 20, [rHLR])
makeSubtask('Sensor data transmission', 7, 11, 10, 0, [rHLR])


#Phys.Design #3 (19-22)
makeTask('PHYSICAL DESIGN', 17, 10, 30, 30, [rVV])
makeSubtask('Design (drawing)', 17, 10, 17, 20, [rVV])
makeSubtask('3D-print', 24, 10, 15, 0, [rVV])
makeSubtask('Mounting', 7, 11, 10, 0, [rVV])



makeTask('MEETINGS AND REPORTS', 17, 10, 40, 0, [rHLR, rVV, rMMM])
#makeSubtask('Prog.Rep.', 4, 10, 2, 100, [rHLR, rVV, rMMM])
#makeSubtask('Meeting', 12, 10, 1, 0, [rHLR, rVV, rMMM])
makeSubtask('Prog.Rep.', 18, 10, 2, 0, [rHLR, rVV, rMMM])
makeSubtask('Meeting', 26, 10, 1, 0, [rHLR, rVV, rMMM])
makeSubtask('Prog.Rep.', 1, 11, 2, 0, [rHLR, rVV, rMMM])
makeSubtask('Meeting', 9, 11, 1, 0, [rHLR, rVV, rMMM])
makeSubtask('Prog.Rep.', 15, 11, 2, 0, [rHLR, rVV, rMMM])
makeSubtask('Meeting', 23, 11, 1, 0, [rHLR, rVV, rMMM])
makeSubtask('Project report', 24, 11, 10, 0, [rHLR, rVV, rMMM])
makeSubtask('Prog.Rep.', 29, 11, 2, 0, [rHLR, rVV, rMMM])
makeSubtask('Meeting', 7, 12, 1, 0, [rHLR, rVV, rMMM])

makeMilestone('Manual control', [tasks[2], tasks[16]])
makeMilestone('Working GUI', [tasks[11]])
makeMilestone('Autonomic task performance', [tasks[6], tasks[7], tasks[22]])
makeMilestone('Finished project report', [tasks[0], tasks[8], tasks[14]])




pRVR = makeProject('Sphero RVR', tasks)
pRVR.make_svg_for_tasks(filename='w44.svg', today=dt.date.today())

if (__name__ == '__main__'):
    print(dt.date.today())