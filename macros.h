#define WAITFORINPUT(){            \
    while(!Serial.available()){};  \
    while(Serial.available()){     \
        Serial.read();             \
    };                             \
}                                  \

/* macros to shift array. Parameters are inclusive. For example:
   int array[] =                  { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
   ARRAYSHIFTUP(array, 2, 6)   => { 1, 2, 3, 3, 4, 5, 6, 7, 9, 10 };
   ARRAYSHIFTDOWN(array, 2, 6) => { 1, 3, 4, 5, 6, 7, 7, 8, 9, 10 }; */

#define ARRAYSHIFTUP(a, lower, upper){            \
    if (lower == 0){                              \
        for (int q = lower; q < upper; q++){      \
            *(a + q) = *(a + q + 1); }            \
    } else{                                       \
        for (int q = lower - 1; q < upper; q++){  \
            *(a + q) = *(a + q + 1); }}}          \

#define ARRAYSHIFTDOWN(a, lower, upper){          \
    if (upper == (sizeof(a)/sizeof(a[0])) - 1){   \
        for (int q = upper - 1; q >= lower; q--){ \
            *(a + q + 1) = *(a + q); }            \
    } else{                                       \
        for (int q = upper; q >= lower; q--){     \
            *(a + q + 1) = *(a + q); }}}          \

#define ARRAYAVERAGE(a, out){                         \
    float sum = 0;                                  \
    for (int i = 0; i < sizeof(a)/sizeof(a[0]); i++){ \
        sum += a[i];}                             \
    out = sum/(sizeof(a)/sizeof(a[0]));}          \

#define CLEARARRAY(a){                                \
    for (int q = 0; q < sizeof(a)/sizeof(a[0]); q++){ \
        a[q] = 0; }}                                  \

#define CLEARSERIAL(){                  \
    while (Serial.available()){         \
        Serial.read();    }}              \

#define PRINTARRAY(a){                                \
    Serial.print('{');                                \
    for (int i = 0; i < sizeof(a)/sizeof(a[0]); i++){ \
        Serial.print(a[i]);                           \
        Serial.print('\t'); }                         \
    Serial.println('}'); }                            \