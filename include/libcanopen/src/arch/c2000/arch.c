#ifdef ARCH_BYTE16

//Defined in the project.
int initReset();

void soft_restart()
{
    initReset();
}
#endif
