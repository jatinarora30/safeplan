
from safeplan.core.stats import Stats
from safeplan.core.visualize import Visualize



sf=Stats("/home/jatinarora/code/safeplan/safeplan/runs/run1.json")
sf.compute()


# vs=Visualize()
# vs.see("/home/jatinarora/code/safeplan/safeplan/runs/run1.json",iterNo=43)