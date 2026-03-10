
### ---------- core rundown ----------- ### 

- configurations are initialized at start, fix them.

- (48-49) dispenser is CM4 and controlled with ethernet now

- (51-53) configure code for setting up Camera object using camera control, consider removing and calling

- (123-130) Find coord of slide to be able to work  within slide frame/verify existing coord of slide

- (133-134) Are these the correct IO assignments for my purposes?

- (306-321) Presumably does not take pictures

- (327) Eventually add AVS detect drop areas functionality

### ---------- one offs ----------- ### 

## config
- dispense configuration file

- configure default speeds in script/config

- consult entire station config

## TCP, coordinate/poses, joint pose
- verify ILCOS syringe and vacuum TCPs

- get TCP offset for camera lens

- pick -> get safe joint pose, get linear pose/coord

- place -> get safe joint pose, get linear pose/coord

## extensions 

- auto focusing with COM port fixed