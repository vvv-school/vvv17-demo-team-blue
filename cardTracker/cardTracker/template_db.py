import os
import os.path as op

import cv2


sift        = cv2.SIFT()
surf        = cv2.SURF(400)
orb         = cv2.ORB(nfeatures = 1000)
detectors   = { 'sift': sift, 'surf': surf, 'orb': orb}


class TemplateDB(object):
    
    
    def __init__(self, path):
        self.path       = path
        self.templates  = {}


        # load templates
        for idx, filename in enumerate(TemplateDB.getFiles(path)):
            template            = {}
            template['file']    = filename
            template['color']   = cv2.imread(filename)
            template['gray']    = cv2.cvtColor( template['color'], cv2.COLOR_BGR2GRAY )
            
            kpt, des  = sift.detectAndCompute(template['gray'], None)
            template['sift']    = (kpt, des)

            kpt, des  = sift.detectAndCompute(template['gray'], None)
            template['surf']    = (kpt, des)

            kpt, des  = sift.detectAndCompute(template['gray'], None)
            template['orb']    = (kpt, des)

            self.templates[idx] = template

            self.printOverview()

    
    def printOverview(self):
        for key in self.templates:
            print key, self.templates[key]['file']



    @staticmethod        
    def getFiles(_path):
        """ This method returns a list of all files within a given \a path. If the path is already a
            file it returns the file contained in a list.
    
        This method does not return hidden files (files starting with a '.').
    
        @return List of files
        """
    
        # test for single file
        if op.isfile(_path):
            return [_path]
    
        elif op.isdir(_path):
            _ls = os.listdir
            _if = op.isfile
            _jo  = op.join
            return [ _jo(_path, f) for f in _ls(_path) if _if(_jo(_path, f)) and not f.startswith('.') ]
    
        return []

        