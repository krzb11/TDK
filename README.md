# Development of a wearable multi-channel EMG acquisition device for control purposes using soft textile technologies

This repository contains the source code, hardware schematics, and other resources related to an embedded hand gesture classification and EMG signal processing device. The project aims to enable real-time classification of hand gestures using electromyography (EMG) signals, processed through a custom embedded system. The trained Support Vector Machine (SVM) model is included to classify the gestures, and data sources used for model training are also provided.

## Project Overview

The system utilizes EMG signals to classify hand gestures for applications such as prosthetic control, rehabilitation, and human-computer interaction. The device includes an embedded system that collects bioelectric signals via electrodes, processes the data, and classifies it in real-time using a machine learning model. The device is designed to be portable and non-invasive, offering easy integration into wearable systems.

### Key Features:
- **Real-time EMG signal processing**: Processing and classification of EMG signals captured from the skin's surface.
- **Embedded system**: A microcontroller-based system for signal acquisition, processing, and classification.
- **Hand gesture classification**: Using a trained Support Vector Machine (SVM) model for classifying different hand gestures.
  
## Contents

- **Source Code**: The implementation of the signal processing pipeline and hand gesture classification using SVM.
- **Hardware Schematics**: Circuit diagrams detailing the hardware design of the embedded system.
- **Trained Model**: The parameters of the SVM model used for gesture classification.
- **Data Source**: Information on the dataset used to train the model.
