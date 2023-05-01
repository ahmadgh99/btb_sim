/* 046267 Computer Architecture - Winter 20/21 - HW #1                  */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <cmath>
#include "vector"
#include "map"

#define VALID_BIT_SIZE 1
#define PC_SIZE 30

int RetardedLOG(int x); // Log2 Replacment since log returns awful shit
int GetBTBSize(); //BTB Size calculate function used in stats
unsigned GetBTBEntryIndex(unsigned pc); // Returns BTB Entry for specified pc (uses my_BTB list bits)
unsigned GetTagFromPC(unsigned pc); // Returns Tag from PC (uses my_BTB tag size)
unsigned GetFSMFromShared(unsigned pc,unsigned History);// Returns FSM Index from pc and history while following L/GShare State

class TargetTableEntery{

public:
    unsigned Tag;
    unsigned Target;
    TargetTableEntery() = default;
    TargetTableEntery(unsigned Tag,unsigned Target):Tag(Tag),Target(Target){}
};

///FSM Table containing FSM Table and default start state. allows easy update on FSM
class FSMTable{

public:
    int* Table;
    unsigned StartState;
    unsigned HistSize;

    FSMTable(unsigned History, unsigned StartState){
        Table = new int[(int)pow(2,History)];
        StartState = StartState;
        HistSize = History;
        for(int i = 0; i < pow(2,History); i++){
            Table[i] = StartState;
        }
    }

    FSMTable(const FSMTable& FS){
        StartState = FS.StartState;
        HistSize = FS.HistSize;
        Table = new int[(int)pow(2,HistSize)];
        for(int i = 0; i < pow(2,HistSize);i++){
            Table[i] = FS.Table[i];
        }
    }

    ~FSMTable(){
        delete[] Table;
    }
    //FSM Update method allows easy update for FSM
    int UpdateState(int Pos,bool Taken,bool Replace = false){
        if(Replace){
            Table[Pos] = StartState;
        }
        if(Taken){
            Increment(Pos);
        }else{
            Decrease(Pos);
        }

        return Table[Pos];
    }

    //Getter Func to Read FSM state
    int ReadState(int Pos){
        return Table[Pos];
    }
private:
    void Increment(int Pos){
        if(Table[Pos] == 3){
            return;
        }
        Table[Pos]++;
    }

    void Decrease(int Pos){
        if(Table[Pos] == 0){
            return;
        }
        Table[Pos]--;
    }

};

/// Local History Table Entry with Target and Tag Entry
class LHist{
public:
    unsigned int History;
    unsigned int MaxHist;
    TargetTableEntery BranchRow;

    unsigned GetTag(){
        return BranchRow.Tag;
    }

    void Update(bool Taken,unsigned int Tag,unsigned target) {
        if (Tag != this->BranchRow.Tag){
            History = 0;
            BranchRow.Tag = Tag;
            BranchRow.Target = target;
        }
        History = (History) % ((unsigned) pow(2, RetardedLOG(MaxHist) - 1));
        History *= 2;
        History += Taken;


    }

    LHist(unsigned int HistSize,unsigned int Tag,unsigned Target){
        History = 0;
        MaxHist = pow(2,HistSize);
        BranchRow.Tag = Tag;
        BranchRow.Target = Target;
    }
};

class BTB{
public:
    // BTB stored settings
    unsigned int BTBSize;
    unsigned int HistorySize;
    unsigned int TagSize;
    unsigned int FSMStartState;
    bool isGlobalHist;
    bool isGlobalTable;
    int SharedState;
    SIM_stats Stats;

    // BTB Stored Branches History Data - Maps' keys is BTB entry bits
    std::map<unsigned ,LHist> History; // Local History (used only for local history)
    std::map<unsigned ,FSMTable> LocalFSMs;

    std::map<unsigned,TargetTableEntery> GlobalTargetTable;


    unsigned int GlobalHistory; // Global History Register (used only for global history)
    FSMTable GlobalFSM; // Global FSM (used only for global tables)

    BTB(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
        bool isGlobalHist, bool isGlobalTable, int Shared): GlobalFSM(FSMTable(historySize,fsmState)){
        this->BTBSize = btbSize;
        this->HistorySize = historySize;
        this->TagSize = tagSize;
        this->FSMStartState = fsmState;
        this->isGlobalHist = isGlobalHist;
        this->isGlobalTable = isGlobalTable;
        this->SharedState = Shared;

        Stats.size = 0;
        Stats.br_num = 0;
        Stats.flush_num = 0;

        GlobalHistory = 0;
    }

    void UpdateGlobalHistory(bool Taken) {

        GlobalHistory = (GlobalHistory) % ((unsigned) pow(2, HistorySize - 1));
        GlobalHistory *= 2;
        GlobalHistory += Taken;
    }

};


BTB* my_BTB; /// Our BTB

int GetBTBSize(){
    if(my_BTB->isGlobalTable && my_BTB->isGlobalHist){
        return my_BTB->BTBSize * (my_BTB->TagSize + VALID_BIT_SIZE + PC_SIZE) + (pow(2, my_BTB->HistorySize+1)) +  my_BTB->HistorySize;
    } else if(my_BTB->isGlobalTable && !my_BTB->isGlobalHist) {
        return my_BTB->BTBSize * (my_BTB->TagSize + VALID_BIT_SIZE + PC_SIZE + my_BTB->HistorySize) + (pow(2, my_BTB->HistorySize+1));
    }else if(!my_BTB->isGlobalTable && my_BTB->isGlobalHist){
        return my_BTB->BTBSize * (my_BTB->TagSize + VALID_BIT_SIZE + PC_SIZE + pow(2, my_BTB->HistorySize+1)) + my_BTB->HistorySize;
    } else {
        return my_BTB->BTBSize * (my_BTB->TagSize + VALID_BIT_SIZE + PC_SIZE + my_BTB->HistorySize + (pow(2, my_BTB->HistorySize+1)));
    }
}

int RetardedLOG(int x){
    int targetlevel = 0;
    while (x >>= 1) ++targetlevel;
    return targetlevel;
}

unsigned GetBTBEntryIndex(unsigned pc){
    unsigned x =  my_BTB->BTBSize -1;
    unsigned y = (pc>>2) & x;
    return y;
}

unsigned GetTagFromPC(unsigned pc){
    auto x = (unsigned)pow(2,(unsigned(my_BTB->TagSize))) -1;
    unsigned y = (pc>>unsigned(RetardedLOG(my_BTB->BTBSize) + 2));
    return y & x;
}

unsigned GetFSMFromShared(unsigned pc,unsigned History){
    if(!my_BTB->isGlobalTable)
        return History;
    switch (my_BTB->SharedState) {
        case 0:
            return History;
        case 1:
            return ((pc>>2)&((unsigned int)pow(2,my_BTB->HistorySize)-1)) ^ History;
        case 2:
            return ((pc>>16)&((unsigned int)pow(2,my_BTB->HistorySize)-1)) ^ History;
    }
    return -1;
}

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
            bool isGlobalHist, bool isGlobalTable, int Shared){

    my_BTB = new BTB(btbSize, historySize, tagSize, fsmState,
             isGlobalHist, isGlobalTable, Shared);

    return 1;
}

bool BP_predict(uint32_t pc, uint32_t *dst){

    //Global History Global Table
    if(my_BTB->isGlobalHist && my_BTB->isGlobalTable){
        // Entry not found in Table
        if (my_BTB->GlobalTargetTable.find(GetBTBEntryIndex(pc))== my_BTB->GlobalTargetTable.end()) {
            *dst = pc + 4;
            return false;
        }
        TargetTableEntery Row = my_BTB->GlobalTargetTable.find(GetBTBEntryIndex(pc))->second;
        if (my_BTB->GlobalFSM.ReadState(GetFSMFromShared(pc, my_BTB->GlobalHistory)) > 1) {
            if (Row.Tag == GetTagFromPC(pc)) { // if FSM is taken and same branch in BTB(tag is equal)
                *dst = Row.Target;
                return true;
            }
        }
        *dst = pc + 4;
        return false;
    //Global History Local Table
    }else if(my_BTB->isGlobalHist && !my_BTB->isGlobalTable){
        // Entry not found in Table
        if(my_BTB->GlobalTargetTable.find(GetBTBEntryIndex(pc)) == my_BTB->GlobalTargetTable.end()){
            *dst = pc+4;
            return false;
        }
        // if FSM is taken and same branch in BTB(tag is equal)
        if(my_BTB->LocalFSMs.find(GetBTBEntryIndex(pc))->second.ReadState(GetFSMFromShared(pc,my_BTB->GlobalHistory)) > 1){
            if(my_BTB->GlobalTargetTable.find(GetBTBEntryIndex(pc))->second.Tag == GetTagFromPC(pc)) {
                *dst = my_BTB->GlobalTargetTable.find(GetBTBEntryIndex(pc))->second.Target;
                return true;
            }
        }
        *dst = pc+4;
        return false;
    }else if(!my_BTB->isGlobalHist && my_BTB->isGlobalTable) { //Local History Global Table
        // Entry not found in Table
        if(my_BTB->History.find(GetBTBEntryIndex(pc)) == my_BTB->History.end()){
            *dst = pc+4;
            return false;
        }
        // if FSM is taken and same branch in BTB(tag is equal)
        if(my_BTB->History.find(GetBTBEntryIndex(pc))->second.GetTag() == GetTagFromPC(pc)){
            unsigned FSMindex = GetFSMFromShared(pc,my_BTB->History.find(GetBTBEntryIndex(pc))->second.History);
            if(my_BTB->GlobalFSM.ReadState(FSMindex) > 1){
                *dst = my_BTB->History.find(GetBTBEntryIndex(pc))->second.BranchRow.Target;
                return true;
            }
        }
        *dst = pc+4;
        return false;
    }else{//Local History Local Table
        // Entry not found in Table
        if(my_BTB->History.find(GetBTBEntryIndex(pc)) == my_BTB->History.end()){
            *dst = pc+4;
            return false;
        }
        // if FSM is taken and same branch in BTB(tag is equal)
        if(my_BTB->History.find(GetBTBEntryIndex(pc))->second.GetTag() == GetTagFromPC(pc)){
            unsigned FSMindex = GetFSMFromShared(pc,my_BTB->History.find(GetBTBEntryIndex(pc))->second.History);
            if(my_BTB->LocalFSMs.find(GetBTBEntryIndex(pc))->second.ReadState(FSMindex) > 1){
                *dst = my_BTB->History.find(GetBTBEntryIndex(pc))->second.BranchRow.Target;
                return true;
            }
        }
        *dst = pc+4;
        return false;
    }
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){

    my_BTB->Stats.br_num++;

    if((pred_dst != targetPc) && taken || (pred_dst != pc+4) && !taken)
        my_BTB->Stats.flush_num++;

    if(my_BTB->isGlobalHist && my_BTB->isGlobalTable){ /// Global History Global Table
        // Entry not found in Table
        if(my_BTB->GlobalTargetTable.find(GetBTBEntryIndex(pc)) == my_BTB->GlobalTargetTable.end()){
            my_BTB->GlobalTargetTable.emplace(GetBTBEntryIndex(pc),TargetTableEntery(GetTagFromPC(pc),targetPc)); // Create new entry in target Table
            my_BTB->GlobalFSM.UpdateState(GetFSMFromShared(pc,my_BTB->GlobalHistory),taken,false); // update global FSM state
            my_BTB->UpdateGlobalHistory(taken);// update global history
        }else{
            //Entry found Tag different
            if(my_BTB->GlobalTargetTable.find(GetBTBEntryIndex(pc))->second.Tag != GetTagFromPC(pc)){
                my_BTB->GlobalTargetTable.erase(GetBTBEntryIndex(pc)); // delete entry
                my_BTB->GlobalTargetTable.emplace(GetBTBEntryIndex(pc),TargetTableEntery(GetTagFromPC(pc),targetPc)); // replace it with the new one
                my_BTB->GlobalFSM.UpdateState(GetFSMFromShared(pc,my_BTB->GlobalHistory),taken,false); // update global FSM state
                my_BTB->UpdateGlobalHistory(taken); // update global history
            }
            else{
                //Entry Found Tag is the same handle updates
                if(pred_dst != targetPc){
                    my_BTB->GlobalTargetTable.find(GetBTBEntryIndex(pc))->second.Target = targetPc;
                }
                my_BTB->GlobalFSM.UpdateState(GetFSMFromShared(pc,my_BTB->GlobalHistory),taken,false); // update global fsm state
                my_BTB->UpdateGlobalHistory(taken); // update global history
            }
        }
    }else if(my_BTB->isGlobalHist && !my_BTB->isGlobalTable){ /// Global History Local Table
        // Entry not found in Table
        if(my_BTB->GlobalTargetTable.find(GetBTBEntryIndex(pc)) == my_BTB->GlobalTargetTable.end()){
            my_BTB->GlobalTargetTable.emplace(GetBTBEntryIndex(pc),TargetTableEntery(GetTagFromPC(pc),targetPc)); // add new entry to local table
            my_BTB->LocalFSMs.emplace(GetBTBEntryIndex(pc), FSMTable(my_BTB->HistorySize,my_BTB->FSMStartState)); // add local FSM table
            my_BTB->LocalFSMs.find(GetBTBEntryIndex(pc))->second.UpdateState(GetFSMFromShared(pc,my_BTB->GlobalHistory),taken,false); // update local fsm state
            my_BTB->UpdateGlobalHistory(taken); // update global history
        }else{
            //Entry found Tag different
            if(my_BTB->GlobalTargetTable.find(GetBTBEntryIndex(pc))->second.Tag != GetTagFromPC(pc)){
                my_BTB->GlobalTargetTable.erase(GetBTBEntryIndex(pc)); // delete the existing entry
                my_BTB->LocalFSMs.erase(GetBTBEntryIndex(pc)); // delete existing FSM state
                my_BTB->GlobalTargetTable.emplace(GetBTBEntryIndex(pc),TargetTableEntery(GetTagFromPC(pc),targetPc)); // add new Entry to local table
                my_BTB->LocalFSMs.emplace(GetBTBEntryIndex(pc), FSMTable(my_BTB->HistorySize,my_BTB->FSMStartState)); // add new FSM
                my_BTB->LocalFSMs.find(GetBTBEntryIndex(pc))->second.UpdateState(GetFSMFromShared(pc,my_BTB->GlobalHistory),taken,false); // update the new FSM
                my_BTB->UpdateGlobalHistory(taken); // update global history
            }else{
                //Entry Found Tag is the same handle updates
                if(pred_dst != targetPc){
                    my_BTB->GlobalTargetTable.find(GetBTBEntryIndex(pc))->second.Target = targetPc;
                }
                my_BTB->LocalFSMs.find(GetBTBEntryIndex(pc))->second.UpdateState(GetFSMFromShared(pc,my_BTB->GlobalHistory),taken,false);
                my_BTB->UpdateGlobalHistory(taken);
            }
        }
    }else if(!my_BTB->isGlobalHist && my_BTB->isGlobalTable){ /// Local History Global Table
        // Entry not found in Table
        if(my_BTB->History.find(GetBTBEntryIndex(pc)) == my_BTB->History.end()){
            my_BTB->History.emplace(GetBTBEntryIndex(pc),LHist(my_BTB->HistorySize,GetTagFromPC(pc),targetPc)); // add new entry to global table
            my_BTB->GlobalFSM.UpdateState(GetFSMFromShared(pc,my_BTB->History.find(GetBTBEntryIndex(pc))->second.History),taken,false); // update FSM state
            my_BTB->History.find(GetBTBEntryIndex(pc))->second.Update(taken, GetTagFromPC(pc),targetPc); // update local history
        }else{
            //Entry found Tag different
            if(my_BTB->History.find(GetBTBEntryIndex(pc))->second.BranchRow.Tag != GetTagFromPC(pc)){
                my_BTB->History.find(GetBTBEntryIndex(pc))->second.Update(false, GetTagFromPC(pc),targetPc); // initialise local history
                my_BTB->GlobalFSM.UpdateState(GetFSMFromShared(pc,my_BTB->History.find(GetBTBEntryIndex(pc))->second.History),taken,false); // update FSM state
                my_BTB->History.find(GetBTBEntryIndex(pc))->second.Update(taken, GetTagFromPC(pc),targetPc); // update local history
            }
            else{
                //Entry Found Tag is the same handle updates
                if(pred_dst != targetPc){
                    my_BTB->History.find(GetBTBEntryIndex(pc))->second.BranchRow.Target = targetPc;
                }
                my_BTB->GlobalFSM.UpdateState(GetFSMFromShared(pc,my_BTB->History.find(GetBTBEntryIndex(pc))->second.History),taken,false); // update FSM state
                my_BTB->History.find(GetBTBEntryIndex(pc))->second.Update(taken, GetTagFromPC(pc),targetPc); // update local history
            }
        }
    }else{///Local History Local Tables
        // Entry not found in Table
        if(my_BTB->History.find(GetBTBEntryIndex(pc)) == my_BTB->History.end()){
            my_BTB->History.emplace(GetBTBEntryIndex(pc),LHist(my_BTB->HistorySize,GetTagFromPC(pc),targetPc)); // add history table to local history row
            my_BTB->LocalFSMs.emplace(GetBTBEntryIndex(pc),FSMTable(my_BTB->HistorySize,my_BTB->FSMStartState)); // add new FSM table
            my_BTB->LocalFSMs.find(GetBTBEntryIndex(pc))->second.UpdateState(GetFSMFromShared(pc,my_BTB->History.find(GetBTBEntryIndex(pc))->second.History),taken,false); // update FSM state
            my_BTB->History.find(GetBTBEntryIndex(pc))->second.Update(taken, GetTagFromPC(pc),targetPc); // update local history
        }else{
            //Entry found Tag different
            if(my_BTB->History.find(GetBTBEntryIndex(pc))->second.BranchRow.Tag != GetTagFromPC(pc)){
                my_BTB->LocalFSMs.erase(GetBTBEntryIndex(pc)); // delete existing FSM table
                my_BTB->LocalFSMs.emplace(GetBTBEntryIndex(pc),FSMTable(my_BTB->HistorySize,my_BTB->FSMStartState)); // add new FSM table
                my_BTB->History.find(GetBTBEntryIndex(pc))->second.Update(false, GetTagFromPC(pc),targetPc); // initialises local history row
                my_BTB->LocalFSMs.find(GetBTBEntryIndex(pc))->second.UpdateState(GetFSMFromShared(pc,my_BTB->History.find(GetBTBEntryIndex(pc))->second.History),taken,false); // Updates Reseted Local FSM Table row
                my_BTB->History.find(GetBTBEntryIndex(pc))->second.Update(taken, GetTagFromPC(pc),targetPc); // Updates Local History Row
            }
            else{
                //Entry Found Tag is the same handle updates
                if(pred_dst != targetPc){
                    my_BTB->History.find(GetBTBEntryIndex(pc))->second.BranchRow.Target = targetPc;
                }
                my_BTB->LocalFSMs.find(GetBTBEntryIndex(pc))->second.UpdateState(GetFSMFromShared(pc,my_BTB->History.find(GetBTBEntryIndex(pc))->second.History),taken,false);
                my_BTB->History.find(GetBTBEntryIndex(pc))->second.Update(taken, GetTagFromPC(pc),targetPc);
            }
        }
    }
}

void BP_GetStats(SIM_stats *curStats){
    my_BTB->Stats.size = GetBTBSize();
    *curStats = my_BTB->Stats ;
    delete my_BTB;
    return;
}