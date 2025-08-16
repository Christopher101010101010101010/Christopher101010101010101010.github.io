from pymongo import MongoClient
from bson.objectid import ObjectId

class AnimalShelter(object):
    """ CRUD operations for Animal collection in MongoDB """

    def __init__(self, userIn, passIn, hostIn, portIn):
        # Initializing the MongoClient. This helps to
        # access the MongoDB databases and collections.
        # This is hard-wired to use the AAC database, the
        # animals collection, and the AAC user.
        # Definitions of the connection string variables are
        # unique to the individual Apporto environment.
        #
        # You must edit the connection variables below to reflect
        # your own instance of MongoDB!
        #
        # Connection Variables
        #
        USER = userIn
        PASS = passIn
        HOST = hostIn
        PORT = portIn
        DB = 'AAC'
        COL = 'animals'
        #
        # Initialize Connection
        #
        self.client = MongoClient('mongodb://%s:%s@%s:%d' % (USER,PASS,HOST,PORT))
        self.database = self.client['%s' % (DB)]
        self.collection = self.database['%s' % (COL)]

# Complete this create method to implement the C in CRUD.
    def create(self, data):

        if data is not None:
            self.database.animals.insert_one(data) #data should be dictionary
            #print(repr(Returnid.inserted_id)) # include 'Returnid =' above to test
            return True

        else:
            raise Exception("Nothing to save, because data parameter is empty")
            return False

# Create method to implement the R in CRUD.
    def read(self, dataPair):
        readResultList = []

        if dataPair is not None:
            mongoCursor =  self.database.animals.find(dataPair) # -initialize mongo cursor 
            #print("retrieved-" + repr(mongoCursor.count())) # for testing cursor
            #print("mongoCursor-\n" + repr(mongoCursor))

            if self.database.animals.count_documents(dataPair) != 0:                                  ##mongoCursor.count() is deprecated as of pymongo v 3.7.0
                for document in mongoCursor:               # -append all matching documents to list
                    readResultList.append(document)

            else:
                raise Exception("Exception: mongoCursor references 0 documents")

        else:
            raise Exception("Exception: read parameter is empty")

        return readResultList

# Create method to implement the U in CRUD
    def update(self, searchPair, updatedPair): #where searchPair, updatedPair are dictionaries
        i = 0
        newVal = { "$set" : updatedPair}
        
        if (searchPair is not None) and (updatedPair is not None):
            mongoCursor = self.database.animals.find(searchPair)

            if self.database.animals.count_documents(dataPair) != 0:                                                #mongoCursor.count() is deprecated as of pymongo v 3.7.0
                for doc in mongoCursor:
                    testVar = self.database.animals.update_one(searchPair, newVal)
                
                #print("mod count:" + repr(testVar.modified_count))
                #if testVar.modified_count == 0:
                #    raise Exception("Exception: document update has failed.")
                    i = i + 1

            else:
                raise Exeption("Exception: mongoCursor references 0 documents")
        else:
            raise Exception("Exception: an update parameter is empty")
       
        return i

# Create method to implement the D in CRUD
    def delete(self, deletePair):
        i = 0

        if deletePair is not None:
            mongoCursor = self.database.animals.find(deletePair)
            if self.database.animals.count_documents(dataPair) != 0:                                                        #mongoCursor.count() is deprecated as of pymongo v 3.7.0
                for doc in mongoCursor:
                    testVar = self.database.animals.delete_one(deletePair)
                
                #print("dlt count:" + repr(testVar.delete_count))
                #if testVar.delete_count == 0:
                #    raise Exception("Exception: document removal has failed.")
                    i = i + 1
            
            else:
                raise Exception("Exception: mongoCursor references 0 documents")
                
        else:
            raise Exception("Exception: delete parameter is empty")

        return i




