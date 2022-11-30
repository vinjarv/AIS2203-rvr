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

#MAIN #7 (0-6)
makeTask('MAIN PROGRAM (C++)', 14, 11, 16, 80, [rHLR, rVV, rMMM])

makeSubtask('Movement', 14, 11, 6, 100, [rVV])
makeSubtask('Manual control (arduino)', 14, 11, 8, 100, [rVV, rMMM])
makeSubtask('Object recognition (openCV)', 14, 11, 13, 100, [rMMM])
makeSubtask('Structure', 14, 11, 13, 80, [rVV])
makeSubtask('Input/output handling', 14, 11, 15, 80, [rVV])
makeSubtask('Lifting and placing', 24, 11, 7, 0, [rVV])
makeSubtask('Program assembly', 30, 11, 4, 20, [rMMM])


#GUI #7 (7-13)
makeTask('GUI (C++, cvUI)', 14, 11, 16, 85, [rMMM])

makeSubtask('Video display', 14, 11, 5, 100, [rMMM])
makeSubtask('Buttons and meters', 14, 11, 5, 100, [rMMM])
makeSubtask('Input handling', 14, 11, 15, 80, [rMMM])        # Structure the inputs
makeSubtask('Error handling', 14, 11, 16, 10, [rMMM])         # Some display of error


#rPI #6 (14-19)

makeTask('RASPBERRY PI (python)', 14, 11, 16, 80, [rHLR])

makeSubtask('Contact and test values', 14, 11, 5, 100, [rVV])
makeSubtask('Video transmission', 14, 11, 5, 100, [rMMM])
makeSubtask('Input handling', 14, 11, 8, 100, [rHLR])
makeSubtask('Sensor data transmission', 14, 11, 14, 70, [rHLR])
makeSubtask('Crash control', 28, 11, 5, 0, [rHLR])
makeSubtask('Program assembly', 30, 11, 4, 20, [rMMM])


#Phys.Design #4 (20-23)
makeTask('PHYSICAL DESIGN', 14, 11, 17, 40, [rHLR, rVV, rMMM])

makeSubtask('Design manual control', 14, 11, 8, 100, [rMMM])
makeSubtask('Design lifting mecanism', 14, 11, 14, 30, [rVV])
makeSubtask('3D-print', 14, 11, 15, 30, [rVV, rMMM])
makeSubtask('Mounting', 2, 12, 3, 20, [rHLR, rVV, rMMM])



makeTask('MEETINGS AND REPORTS', 14, 11, 22, 50, [rHLR, rVV, rMMM])

makeSubtask('Prog.Rep.', 15, 11, 2, 100, [rHLR, rVV, rMMM])
makeSubtask('Meeting', 23, 11, 1, 0, [rHLR, rVV, rMMM])
makeSubtask('Project report', 1, 12, 7, 0, [rHLR, rVV, rMMM])
makeSubtask('Prog.Rep.', 29, 11, 2, 0, [rHLR, rVV, rMMM])
makeSubtask('Meeting', 7, 12, 1, 0, [rHLR, rVV, rMMM])

makeMilestone('Autonomic operations', [tasks[1], tasks[3], tasks[5], tasks[15]])
makeMilestone('Finished project report', [tasks[0], tasks[8], tasks[13], tasks[20], tasks[28]])




pRVR = makeProject('Sphero RVR', tasks)
pRVR.make_svg_for_tasks(filename='w48.svg', today=dt.date.today())

if (__name__ == '__main__'):
    print(dt.date.today())




