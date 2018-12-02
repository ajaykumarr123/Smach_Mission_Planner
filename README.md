# Smach_Mission_Planner-
Mission Planner For AUV using Smach




Instructions:-
Clone this Repo and create a new branch ,edit commit and send a pull request  
Intial Task 

1.Call Web camera Node To publish Image data to a particular Topic
2.Create a Action Server that subscribes that Particular topic applies HSV thesholing for Red colour and apply rectangular bounding box  and publish its center  to another Topic
3.Create Client which calls the above Server ( to subscribe and publish the center point of blog) 
4.Integrate Client with SMACH . 





You need to learn-
Action Server
Smach
image_transport 


Commit #1-Test code added,kraken_msgs(kraken_reboot_stack) must be  catkinised before running the code  