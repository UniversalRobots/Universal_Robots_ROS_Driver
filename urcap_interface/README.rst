URCap-ROS interface
===================

Available components
--------------------


ROS-Topic subscriber
~~~~~~~~~~~~~~~~~~~~

Backend (ROS side) creates subscriber if program node in program

 - ProgramNode
   - topic selector
   - Data handling
   - timeout

   Runtime error if timeout hit
   Runtime error if no connection to backend

   on enter request data from backend

.. todo::
  mockup


ROS-Topic publisher
~~~~~~~~~~~~~~~~~~~

Backend (ROS side) creates publisher if program node in program

Backend defines supported topic types (can be extended by user)

 - ProgramNode
   - topic definition
   - datatype selector
     -> Data template

   Runtime error if no connection to backend

   on enter send data to backend

.. todo::
   - mockup


ROS-Service caller
~~~~~~~~~~~~~~~~~~

Backend (ROS side) creates service client if program node in program

 - ProgramNode
   - topic selector
   - Data handling

   Runtime error if service unavailable
   Runtime error if no connection to backend

   on enter request data from backend -> call service, send back response

.. todo::
  mockup


Topic selector
~~~~~~~~~~~~~~

Available topics should be forwarded to UR side (maybe with a filter on ROS side installed). Users should then be able to setup filters
in the URCap Installation Node to reduce the number of available topics when building programs. We can then either use dropdown
menus or input text fields, let’s see what works best.


Polyscope functions
~~~~~~~~~~~~~~~~~~~

Promote certain script functions to PolyScope functions so they can be used inside of if statements, for example. (e.g. querying a
parameter or a value from a topic)


Action wrapper
~~~~~~~~~~~~~~

 * Actions should be exposed manually via configuration together with some meta information
    * Does this action require external control?
    * A short description of the task

 - ProgramNode
   - action selector
   - Data template

   Runtime error if action unavailable
   Runtime error if no connection to backend

   on enter call action via backend, visualize feedback on program node

.. todo::
   mockup

External control
~~~~~~~~~~~~~~~~

Keep the current external_control as another ProgramNode as is

Architecture
------------

* communication between backend and URCap?


Backend
~~~~~~~
* Action configuration
* Type configuration
* Subscriber relay
* Service client relay
* Publisher relay

URCap
~~~~~
* How does installation node look like?
* What kind of script contributions do we generate?
* How do the GUI elements look like?
