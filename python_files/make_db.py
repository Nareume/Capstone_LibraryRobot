import pandas as pd
import numpy as np
from IPython.display import display

label_id = np.arange(1, 301)
status = np.full(300, 'returned')
location = np.zeros((300, 2))


location[:36, 0] = 1.3
location[36:72, 0] = 2.2
location[72:100, 0] = 3.1
location[100:136, 0] = 1.3
location[136:172, 0] = 2.2
location[172:200, 0] = 3.1
location[200:236, 0] = 1.3
location[236:272, 0] = 2.2
location[272:, 0] = 3.1
location[:100, 1] = -0.5
location[100:200, 1] = 1
location[200:, 1] = 2.5

db_dict = {"id" : label_id, "status" : status,
           "pose_x" : location[:,0], "pose_y" : location[:,1],
}

library_db = pd.DataFrame(db_dict)
display(pd.DataFrame(library_db))

library_db.to_csv('python_files/library_db.csv', sep=',', na_rep='Nan')