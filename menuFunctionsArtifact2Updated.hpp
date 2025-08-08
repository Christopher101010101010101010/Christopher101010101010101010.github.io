// header file contains functions relating to menu
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include "BinarySearchTree.hpp"
#include <conio.h>          // _getch() host-lib

using namespace std;

//Inline function named filePathFunction returning string, noted as inline
// for example file enter "courseInfo.txt"
inline string filePathFunction() {
    //Define string named filePath
    char  userFilePath[256];
    //Output prompt for user to enter file name. Set input to filePath
    cout << "Enter file name- \n(include file extension)" << endl;;
    cin.getline(userFilePath, 256);

    //Set filePath to "./" and filePath's current state
    string filePath;
    filePath.append('.' + '/' + userFilePath);
    return filePath;
}

//Inline function named loadCourseInfoBST of void return with BinarySearchTree reference named courseBST and string named dataFilePath
inline void loadCourseInfoBST(BinarySearchTree &courseBST, string dataFilePath) {
    //Define ifstream handle named dataInHandle, string named lineStr and size_t named prerequisiteAdditionalFound
    ifstream dataInHandle;
    string lineStr;
    size_t prerequisiteAdditionalFound;

    //Check section of code for errors
    try {
    
        //open dataFilePath's file and bind to dataInHandle, copy first line of dataFilePath's file to lineStr
        dataInHandle.open(dataFilePath);
        getline(dataInHandle, lineStr);

        //While dataInHandle is not returning end of file flag
        while (!dataInHandle.eof()) {
            
            //Define Course named newCourse, copy next line of file through dataInHandle to lineStr
            Course * newCourse = new Course;
            getline(dataInHandle, lineStr);
            
            //Initialize integer named positionSplit1 to index of first "," and positionSplit2 to index of second ","
            int positionSplit1 = lineStr.find(",");
            int positionSplit2 = lineStr.find(",", positionSplit1 + 1);
            
            //Set newCourse's courseNumber member to string before first comma and newCourse's name to string between first and second commas
            newCourse->courseNumber = lineStr.substr(0, positionSplit1);
            newCourse->name = lineStr.substr(positionSplit1 + 1, positionSplit2-1);

            //Set positionSplit1 to occurence of next comma and prerequisiteAdditionalFound to value returned by lineStr seeking another occurence of ","
            positionSplit1 = lineStr.find(",", positionSplit2 + 1);
            prerequisiteAdditionalFound = lineStr.find(",", positionSplit1 + 1);

            //If prerequisiteAdditionalFound is npos
            if (prerequisiteAdditionalFound == string::npos)
            {
                //Set newCourse's prerequisiteListRoot's prerequisiteNumber to lineStr's positionSplit1 to end of string
                delete newCourse->prerequisiteListRoot;
                newCourse->prerequisiteListRoot->prerequisiteNumber = lineStr.substr(positionSplit1 + 1);
            }
            else {
            
                //Set positionSplit2 to index of next occurence of comma and set newCourse's prerequisiteListRoot's prerequisiteNumber to lineStr
                //between positionSplit1 and positionSplit2
                positionSplit2 = lineStr.find(",", positionSplit1 + 1);
                delete newCourse->prerequisiteListRoot;
                newCourse->prerequisiteListRoot->prerequisiteNumber = lineStr.substr(positionSplit1 + 1, positionSplit2 - 1);

                //Set prerequisiteAdditionalFound to return of lineStr seeking next comma
                prerequisiteAdditionalFound = lineStr.find(",", positionSplit2 + 1);

                //While prerequisiteAdditionalFound is not npos
                while (prerequisiteAdditionalFound != string::npos) {
                
                    //Initialize prerequisiteListNode pointer named nextPrerequisite to null
                    prerequisiteListNode* nextPrerequisite = nullptr;

                    //Set positionSplit1 to lineStr seeking index of next comma
                    positionSplit1 = lineStr.find(",", positionSplit2 + 1);

                    //Set prerequisiteAdditionalFound to value returned by lineStr seeking next comma
                    prerequisiteAdditionalFound = lineStr.find(",", positionSplit1 + 1);
                    
                    //If prerequisiteAdditionalFound is npos
                    if (prerequisiteAdditionalFound == string::npos)
                        
                        //Set nextPrerequisite's prerequisiteNumber to lineStr from last comma to end of string
                        nextPrerequisite->prerequisiteNumber = lineStr.substr(positionSplit1 + 1);
                    else {
                    
                        //Set positionSplit2 to index of next comma in lineStr
                        positionSplit2 = lineStr.find(",", positionSplit1 + 1);
                        //Set nextPrerequisite's prerequisiteNumber to lineStr between positionSplit1 and positionSplit2
                        nextPrerequisite->prerequisiteNumber = lineStr.substr(positionSplit1 + 1, positionSplit2 - 1);
                    }

                    //Set nextPrerequisite's next pointer to newCourse's prerequisiteListRoot's next pointer
                    nextPrerequisite->next = newCourse->prerequisiteListRoot->next;
                    //Set newCourse's prerequisiteListRoot's next pointer to nextPrerequisite
                    newCourse->prerequisiteListRoot->next = nextPrerequisite;
                }
            }

            //Call courseBST's Insert function with newCourse as parameter
            courseBST.Insert(*newCourse);
        }

        //close dataInHandle bound to dataFilePath's file
        dataInHandle.close();
    }
    //Should any errors be reported by previous code
    catch (exception& e)
    {
        //Output error code
        cout << "\n\nError code- " << e.what() << endl;
    }
}

//Inline function named printObjectBST of void return with string named courseNumberKey and BinarySearchTree named bst
inline void printObjectBST(string courseNumberKey, BinarySearchTree bst) {
    //Initialize Course named aCourse to return of bst's Search function with courseNumberKey as parameter
    Course aCourse = bst.Search(courseNumberKey);

    //If aCourse's courseNumber is equivalent to courseNumberKey
    if (aCourse.courseNumber == courseNumberKey) {
        
        //Output aCourse's courseNumber ", " aCourse's name ", "
        cout << aCourse.courseNumber << ", " << aCourse.name << ", ";
        
        //Call printCoursePrerequisiteList with aCourse as parameter
        printCoursePrerequisiteList(aCourse);
    }
    else {
        
        //Output error statement
        cout << "\n\nError- course returned by binary search tree does not match course number searched." << endl;
    }
}

//Inline function named menuFunction returning bool with BinarySearchTree reference named courseBST as parameter
inline bool menuFunction(BinarySearchTree &courseBST) {
    //Define short integer named userIn, strings named courseNumberInput and filePath
    string courseNumberInput, filePath;
    short userIn;
    
    char inputBuffer;
    int i = 0;

    //Output Menu options and prmpt user for input
    cout << "Menu-\n1.Load Data Structure:\n2.Print Course List:\n3.Print Course:\n4.Exit" << endl;
    
    //UPDATE BLOCK 7-26-2025
    // 
    //get keyboard input and store to inputBuffer
    inputBuffer = _getch();

    // while the user has not entered an integer char 0-9, acquire user input
    while (inputBuffer < '0' || inputBuffer > '9') {    
                
        //get keyboard input and store to inputBuffer
        inputBuffer = _getch();
    
    }

    // once the user has entered an integer char , 0-9, display the char in the console
    cout.put(inputBuffer);

    //if the user has input an integer char, 1-3, assign this integer to the userIn variable
    if (inputBuffer > '0' && inputBuffer < '4') {
        userIn = inputBuffer - '0';
    }
    else {                              // otherwise assign the userIn variable the inconsequential integer 5
        userIn = 5;
    }

    //END UPDATED BLOCK 7-26-2025
    
    //Initiate switch controlled by userIn
    switch (userIn) {
    
        //When userIn is 1
        case 1:
            //Set filePath to name of file in executable's current directory and call loadCourseInfoBST with courseBST and filePath
            filePath = filePathFunction();
            loadCourseInfoBST(courseBST, filePath);
            break;

        //When userIn is 2
        case 2:
            //call data structure instance's print all function
            courseBST.printAllCourseBST();
            break;

        //When userIn is 3
        case 3:
            //Prompt user for course number to search for
            cout << "Enter course number: ";

            //UPDATE 7-26-2025
            //
            i = 0;
            while (i < courseNumberInput.max_size() && i > -1) {          //while input counter named i is less than the maximum allowed string length
                                                                          // and the counter i is greater than -1
                inputBuffer = _getch();                           //assign a single char of keyboard input to inputBuffer
                
                if (inputBuffer > ' ' && inputBuffer < '~') {    //if input is a standard symbol, alphabetic, or numeric add to the string
                    cout.put(inputBuffer);                      //display keyboard input to console
                    courseNumberInput[i] = inputBuffer;         //concatenate inputBuffer's char to the i-th index of the string named courseNumberInput
                    i++;
                    cout.put(inputBuffer);
                }
                
                else if (inputBuffer == 8) {                    //if the input is a backspace then decrement to the previous string index,
                    i--;                                        // nullify the char assigned to this index.
                    courseNumberInput[i] = '\0';
                    cout.put('\b');
                }
               
                else if (inputBuffer == '\n') {                   //if the input is a carriage return(Enter key) then exit the while loop
                    i = -1;
                }
               
            }

            //UPDATE END

            //call print object with data structure instance and courseNumberInput
            printObjectBST(courseNumberInput, courseBST);
            break;

        //When userIn is 4
        case 4:
            // return true, that will set exitCondition to true ending the program
            return true;
            break;

        //When userIn is not a defined integer
        default:
            //Output Error statement
            cout << "Invalid input- " << endl << userIn << endl;

    }

    //return false, signifying that user does not want to exit
    return false;
}
