
#include <cstdlib>
#include <iostream>
#include <vector>
#include "beacon.h"
#include <iomanip>
#include <string>
using namespace std;

BeaconList::BeaconList(){
    head=NULL;
    curr=NULL;
    temp=NULL;
}


void BeaconList::AddBeacon(int b_idMsg, string b_typeNode,string b_status,simtime_t b_time, simtime_t b_timeRx, int b_idVehicle,double b_s, double b_pX, double b_pY, double b_pZ,
            double b_disJunction, double b_disToSource, double b_disToDestination,simtime_t b_idleTime, double b_abe, double b_angle){
    beaconPtr n = new beacon;
    n->next =NULL;
    n->idMsg=b_idMsg;
    n->typeNode=b_typeNode;
    n->status=b_status;
    n->time=b_time;
    n->timeRx=b_timeRx;
    n->idVehicle=b_idVehicle;
    n->s=b_s;
    n->pX=b_pX;
    n->pY=b_pY;
    n->pZ=b_pZ;
    n->disJunction=b_disJunction;
    n->disToSource=b_disToSource,
    n->disToDestination=b_disToDestination;
    n->idleTime=b_idleTime;
    n->abe=b_abe;
    n->angle=b_angle;



    if(head != NULL){
        curr = head;
        while(curr->next != NULL){
            curr = curr->next;

        }
        curr->next=n;
    }
    else{
        head=n;
    }
}

void BeaconList::DeleteBeacon(int delb_idVehicle){
    beaconPtr delPtr = NULL;
    temp = head;
    curr = head;
    while(curr !=NULL && curr->idVehicle != delb_idVehicle){
        temp = curr;
        curr = curr->next;
    }
    if(curr == NULL){
        EV << delb_idVehicle << " was not in the beacon List" <<std::endl;
        delete delPtr;
    }
    else{
        delPtr = curr;
        curr = curr->next;
        temp->next = curr;
        if(delPtr==head){
            head= head->next;
            temp = NULL;
        }


        delete delPtr;
        EV << delb_idVehicle << " This beacon was deleted" <<std::endl;
    }

}

void BeaconList::PrintBeacons(){
    curr = head;
    EV<<"----------------------------------------------------------------------------------------------------------------------------------------------"<<endl;
    EV<<setw(40)<<"Neighbor Table"<<endl;
    EV<<"----------------------------------------------------------------------------------------------------------------------------------------------"<<endl;
    EV<<"ArrivalTime"<<setw(20)<<"Id"<<setw(15)<<"Type"<<setw(15)<<"Status"<<setw(15)<<"Speed"<<setw(15)<<"CoordX"<<setw(15)<<"CoordY"<<setw(15)<<"DistanceToRSU"<<endl;
    while(curr != NULL)
    {
        EV<<curr->time<<setw(15)<<curr->idVehicle<<setw(15)<<curr->typeNode<<setw(15)<<curr->status<<setw(15)<<curr->s<<setw(15)<<curr->pX<<setw(15)<<curr->pY<<setw(15)<<curr->disToSource<<endl;
        curr = curr->next;

    }
    EV<<"----------------------------------------------------------------------------------------------------------------------------------------------"<<endl;

}


int BeaconList::CounterBeacons(){
    curr = head;
    int counter =0;

    while(curr != NULL)
    {
        curr = curr->next;
        counter++;


    }
    return counter;
}

bool BeaconList::SearchBeacon(int delb_idVehicle){
    bool foundBeacon;
    beaconPtr delPtr = NULL;
    temp = head;
    curr = head;
    while(curr !=NULL && curr->idVehicle != delb_idVehicle){
        temp = curr;
        curr = curr->next;
    }
    if(curr == NULL){
        EV << delb_idVehicle << " was not in the beacon List" <<std::endl;
        foundBeacon = false;
    }
    else{
        EV << delb_idVehicle << " This beacon was found!!" <<std::endl;
        foundBeacon = true;
    }
    delete delPtr;
    return foundBeacon;

}


void BeaconList::UpdateBeacon(int b_idMsg, std::string b_typeNode,string b_status, simtime_t b_time, simtime_t b_timeRx,int b_idVehicle,double b_s, double b_pX, double b_pY, double b_pZ,
            double b_disJunction, double b_disToSource, double b_disToDestination, simtime_t b_idleTime, double b_abe, double b_angle){
    beaconPtr n = new beacon;
    n =head;

    while(n != NULL)
    {

        if(n->idVehicle == b_idVehicle)
        {
            n->idMsg=b_idMsg;
            n->typeNode=b_typeNode;
            n->status=b_status;
            n->time=b_time;
            n->timeRx=b_timeRx;
            n->idVehicle=b_idVehicle;
            n->s=b_s;
            n->pX=b_pX;
            n->pY=b_pY;
            n->pZ=b_pZ;
            n->disJunction=b_disJunction;
            n->disToSource=b_disToSource;
            n->disToDestination=b_disToDestination;
            n->idleTime= b_idleTime;
            n->abe=b_abe;
            n->angle=b_angle;
            return;
        }
        else
            n = n->next;

    }

}



void BeaconList::SortBeacons(){
    bool changeMade;
    do{
        changeMade = false;
        curr = head;
        while( curr != NULL ){
            temp = curr;
            curr = curr->next;

            if( curr && curr->time > temp->time ){
                changeMade = true;
                swap( temp->time, curr->time );
                swap( temp->typeNode, curr->typeNode );
                swap( temp->status, curr->status );
                swap( temp->idVehicle, curr->idVehicle);
                swap( temp->s, curr->s);
                swap( temp->pX, curr->pX);
                swap( temp->pY, curr->pY);
                swap( temp->disToSource, curr->disToSource);
                swap( temp->disToDestination, curr->disToDestination);
                swap( temp->idleTime, curr->idleTime);
                swap( temp->abe, curr->abe);
                swap( temp->angle, curr->angle);
            }
        }
    } while( changeMade );
}



int BeaconList::PurgeBeacons(double b_ttl){

    curr =head;
    double counter=0;
    double ttl=simTime().dbl();
    while(curr != NULL && curr->next !=NULL)
       {
           if(curr->next->time < ttl-b_ttl)
           {
              temp = curr->next;
              curr->next= temp->next;
              free(temp);
              counter++;
           }
           else
               curr= curr->next;

       }

      EV  << " This beacon list was purged. "<<counter<<" has been deleted" <<std::endl;
      return counter;

}



