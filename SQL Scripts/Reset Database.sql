/* RESET DATABASE */
delete from AttackBattles;
delete from PracticeTouge;
delete from CMBattles;
delete from OwnedCars;
delete from PlayerSettings;
delete from Groups where id != 0;
delete from Players where id != 0;
/* WARNING!!!!! */