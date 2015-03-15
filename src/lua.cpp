void createFile(string robotName){
	ofstream myFile;
	myFile.open("simulation.lua");

	myFile << "wc = rws.getRobWorkStudio():getWorkCell()" << endl;
	myFile << "state = wc:getDefaultState()" << endl;
	myFile << "device = wc:findDevice(\"" << robotName << "\")" << endl;
	myFile << "gripper = wc:findFrame(\"ToolMount\");" << endl;
	myFile << "bottle = wc:findFrame(\"Bottle\");" << endl;
	myFile << "table = wc:findFrame(\"Table\");" << endl << endl;
	myFile << "function setQ(q)" << endl;
	myFile << "qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])" << endl;
	myFile << "device:setQ(qq,state)" << endl;
	myFile << "rws.getRobWorkStudio():setState(state)" << endl;
	myFile << "rw.sleep(1)" << endl;
	myFile << "end" << endl << endl;
	myFile << "function attach(obj, tool)" << endl;
	myFile << "rw.gripFrame(obj, tool, state)" << endl;
	myFile << "rws.getRobWorkStudio():setState(state)" << endl;
	myFile << "rw.sleep(0.01)" << endl;
	myFile << "end" << endl << endl;

	myFile.close();
}

/*****************************************************************************************************************************/
/******Function: addPosition												     */
/******Inputs: next state vector of the robot									             */
/******Outputs: input argument for the LUA function setQ								     */
/******This function calls the setQ function in the LUA script to move the robot to its next configuration updating its state*/
/*****************************************************************************************************************************/
void addPosition(vector<Q> q){
	ofstream myFile;
	myFile.open("simulation.lua",ios::app);
	
	for(size_t i=0; i<q.size(); i++){
		myFile <<"setQ({"<<q[i][0]<<","<<q[i][1]<<","<<q[i][2]<<","<<q[i][3]<<","<<q[i][4]<<","<<q[i][5]<<"})"<<endl;

	myFile.close();
}