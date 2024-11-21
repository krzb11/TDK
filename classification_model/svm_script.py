import csv
import numpy as np
from sklearn import svm
from sklearn.svm import LinearSVC
from sklearn.multiclass import OneVsRestClassifier
import random
import math

with open('train_X.csv',
          newline='') as train_x_csvfile:
    train_x_reader = csv.reader(train_x_csvfile, delimiter=';')
    rows = list(train_x_reader)
    train_x = np.zeros([train_x_reader.line_num, len(rows[0])])

    for i, row in enumerate(rows):
        for e in range(len(row)):
            train_x[i, e] = float(row[e])

with open('train_Y.csv',
          newline='') as train_y_csvfile:
    train_y_reader = csv.reader(train_y_csvfile, delimiter=';')
    rows = list(train_y_reader)
    train_y = []

    for row in enumerate(rows):
        train_y.append(row[1][0])

with open('test_X.csv',
          newline='') as validation_x_csvfile:
    validation_x_reader = csv.reader(validation_x_csvfile, delimiter=';')
    rows = list(validation_x_reader)
    validation_x = np.zeros([validation_x_reader.line_num, len(rows[0])])

    for i, row in enumerate(rows):
        for e in range(len(row)):
            validation_x[i, e] = float(row[e])

with open('test_Y.csv',
          newline='') as validation_y_csvfile:
    validation_y_reader = csv.reader(validation_y_csvfile, delimiter=';')
    rows = list(validation_y_reader)
    validation_y = []

    for row in enumerate(rows):
        validation_y.append(row[1][0])

classes = list(set(train_y))
classes.reverse()
classifiers = []

for cls in classes:
    rest_indexes = [i for i, val in enumerate(train_y) if val != cls]
    validation_rest_indexes = [i for i, val in enumerate(validation_y) if val != cls]

    cls_train_y = np.array(train_y)
    cls_validation_y = np.array(validation_y)

    for i in rest_indexes:
        cls_train_y[i] = 'other'
    for i in validation_rest_indexes:
        cls_validation_y[i] = 'other'

    clf = svm.SVC(kernel='poly', degree=2, gamma='auto', coef0=0.1)
    clf.fit(train_x, cls_train_y)

    y_pred = clf.predict(validation_x)
    print("'%s' classifier accuracy: %.3f" % (cls, clf.score(validation_x, cls_validation_y)))

    classifiers.append(clf)

for i, cls in enumerate(classes):
    clf = classifiers[i]
    supportShape = clf.support_vectors_.shape
    nbSupportVectors = supportShape[0]
    vectorDimensions = supportShape[1]
    dualCoefs = clf.dual_coef_
    dualCoefs = dualCoefs.reshape(nbSupportVectors)
    supportVectors = clf.support_vectors_
    supportVectors = supportVectors.reshape(nbSupportVectors * vectorDimensions)

    print("'%s' classifier parameters:" % cls)
    print("uint32_t %s_nbSupportVectors = %d;" % (cls.replace(' ', '_'), nbSupportVectors))
    print("uint32_t %s_degree = %d;" % (cls.replace(' ', '_'), clf.degree))
    print("float32_t %s_coef0 = %ff;" % (cls.replace(' ', '_'), clf.coef0))
    print("float32_t %s_gamma = %ff;" % (cls.replace(' ', '_'), clf._gamma))
    print("float32_t %s_intercept = %ff;" % (cls.replace(' ', '_'), clf.intercept_[0]))
    s = np.array(classifiers[i].classes_ != 'other')*(i+1)
    print("int32_t %s_classes[] = {%d, %d};" % (cls.replace(' ', '_'), s[0], s[1]))
    s = "float32_t " + cls.replace(' ', '_') + "_dualCoefs[] = {"
    for c, dualCoef in enumerate(dualCoefs):
        if c == dualCoefs.shape[0]-1:
            s = s + str(dualCoef) + "f};"
        else:
            s = s + str(dualCoef) + "f, "
    print(s)
    s = "float32_t " + cls.replace(' ', '_') + "_supportVectors[] = {"
    for c, supportVector in enumerate(supportVectors):
        if c == supportVectors.shape[0]-1:
            s = s + str(supportVector) + "f};"
        else:
            s = s + str(supportVector) + "f, "
    print(s)