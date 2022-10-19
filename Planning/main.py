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
makeTask('MAIN PROGRAM (C++)', 21, 9, 48, 5, [rHLR, rVV, rMMM])
makeSubtask('Structure', 22, 9, 20, 10, [rVV])
makeSubtask('Manual control (xInput)', 28, 9, 18, 20, [rVV])
makeSubtask('Input handling', 10, 10, 10, 0, [rVV])
makeSubtask('Crash control', 17, 10, 8, 0, [rVV])
makeSubtask('Object recognition (openCV)', 19, 10, 15, 0, [rVV])
makeSubtask('Movement', 31, 10, 15, 0, [rVV])
makeSubtask('Lifting and placing', 7, 11, 10, 0, [rVV])


#GUI #6 (8-13)
makeTask('GUI (C++, Qt)', 21, 9, 48, 0, [rMMM])
makeSubtask('Test values', 22, 9, 22, 0, [rMMM])
makeSubtask('Layout', 7, 10, 16, 0, [rMMM])
makeSubtask('Video display', 21, 10, 7, 0, [rMMM])
makeSubtask('Buttons and meters', 31, 10, 10, 0, [rMMM])
makeSubtask('Error handling', 7, 11, 10, 0, [rMMM])


#rPI #5 (14-18)
makeTask('RASPBERRY PI (python)', 21, 9, 48, 0, [rHLR])
makeSubtask('Contact and test values', 22, 9, 15, 0, [rHLR])
makeSubtask('Input handling', 10, 10, 10, 0, [rHLR])
makeSubtask('Video transmission', 14, 10, 6, 20, [rHLR])
makeSubtask('Sensor data transmission', 21, 10, 10, 0, [rHLR])


#Phys.Design #3 (19-22)
makeTask('PHYSICAL DESIGN', 21, 9, 48, 0, [rVV])
makeSubtask('Design (drawing)', 22, 9, 22, 20, [rVV])
makeSubtask('3D-print', 17, 10, 8, 0, [rVV])
makeSubtask('Mounting', 21, 10, 11, 0, [rVV])




makeTask('MEETINGS AND REPORTS', 21, 9, 58, 0, [rHLR, rVV, rMMM])
makeSubtask('Prog.Rep.', 4, 10, 2, 100, [rHLR, rVV, rMMM])
makeSubtask('Meeting', 12, 10, 1, 0, [rHLR, rVV, rMMM])
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
pRVR.make_svg_for_tasks(filename='w42.svg', today=dt.date.today())

if (__name__ == '__main__'):
    print(dt.date.today())