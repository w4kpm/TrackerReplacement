const uint8_t star[64][32] = {{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x2, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x9, 0x90, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xE, 0xE0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5E, 0xE5, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xCE, 0xEC, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xDD, 0xDD, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5, 0xDD, 0xDD, 0x50, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xC, 0xCC, 0xCC, 0xC0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xC, 0xCB, 0xBC, 0xC0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x6B, 0xBB, 0xBB, 0xB6, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xBB, 0xBB, 0xBB, 0xBB, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xBA, 0xAA, 0xAA, 0xAB, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5, 0xAA, 0xAA, 0xAA, 0xAA, 0x50, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xA, 0x99, 0x99, 0x99, 0x99, 0xA0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x9, 0x99, 0x99, 0x99, 0x99, 0x90, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x58, 0x88, 0x88, 0x88, 0x88, 0x85, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x98, 0x88, 0x87, 0x78, 0x88, 0x89, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x87, 0x77, 0x77, 0x77, 0x77, 0x78, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x40, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8, 0x77, 0x66, 0x66, 0x66, 0x66, 0x77, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x7, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x70, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x36, 0x66, 0x55, 0x55, 0x55, 0x55, 0x66, 0x63, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0xAE, 0xEE, 0xED, 0xDC, 0xCB, 0xBA, 0xA9, 0x99, 0x88, 0x87, 0x76, 0x67, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x76, 0x67, 0x78, 0x88, 0x99, 0x9A, 0xAB, 0xBC, 0xCD, 0xDE, 0xEE, 0xEA, 0x0 },
{0x0, 0xB, 0xEE, 0xDD, 0xCC, 0xBB, 0xBA, 0xA9, 0x98, 0x87, 0x77, 0x66, 0x70, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x7, 0x66, 0x77, 0x78, 0x89, 0x9A, 0xAB, 0xBB, 0xCC, 0xDD, 0xEE, 0xB0, 0x0 },
{0x0, 0x0, 0x8E, 0xDD, 0xCC, 0xBB, 0xAA, 0x99, 0x88, 0x77, 0x76, 0x66, 0x60, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x6, 0x66, 0x67, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xE8, 0x0, 0x0 },
{0x0, 0x0, 0x3, 0xDC, 0xCB, 0xBB, 0xAA, 0x99, 0x88, 0x77, 0x76, 0x65, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x56, 0x67, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xBC, 0xCD, 0x30, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0xC, 0xCB, 0xBB, 0xAA, 0x99, 0x88, 0x77, 0x66, 0x56, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x65, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xBC, 0xC0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x9C, 0xBA, 0xA9, 0x98, 0x87, 0x77, 0x66, 0x56, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x65, 0x66, 0x77, 0x78, 0x89, 0x9A, 0xAB, 0xC9, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x3, 0xBA, 0xA9, 0x98, 0x87, 0x76, 0x65, 0x50, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5, 0x56, 0x67, 0x78, 0x89, 0x9A, 0xAB, 0x30, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0xB, 0xA9, 0x98, 0x87, 0x76, 0x65, 0x60, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x6, 0x56, 0x67, 0x78, 0x89, 0x9A, 0xB0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x89, 0x98, 0x87, 0x76, 0x65, 0x70, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x7, 0x56, 0x67, 0x78, 0x89, 0x98, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x98, 0x77, 0x76, 0x65, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x56, 0x67, 0x77, 0x89, 0x10, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xA, 0x87, 0x76, 0x66, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x66, 0x67, 0x78, 0xA0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x68, 0x76, 0x67, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x76, 0x67, 0x86, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x86, 0x60, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x6, 0x68, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8, 0x60, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x6, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x67, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x76, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x9, 0x66, 0x66, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x66, 0x66, 0x90, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8, 0x76, 0x65, 0x70, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x7, 0x56, 0x67, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x7, 0x76, 0x66, 0x56, 0x20, 0x0, 0x0, 0x0, 0x0, 0x2, 0x65, 0x66, 0x67, 0x70, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x97, 0x77, 0x66, 0x65, 0x57, 0x0, 0x0, 0x0, 0x0, 0x75, 0x56, 0x66, 0x77, 0x79, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x98, 0x77, 0x76, 0x66, 0x55, 0x70, 0x0, 0x0, 0x7, 0x55, 0x66, 0x67, 0x77, 0x89, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x98, 0x87, 0x77, 0x66, 0x66, 0x56, 0x40, 0x4, 0x65, 0x66, 0x66, 0x77, 0x78, 0x89, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8, 0x98, 0x88, 0x77, 0x77, 0x66, 0x66, 0x68, 0x86, 0x66, 0x66, 0x77, 0x77, 0x88, 0x89, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xA, 0x99, 0x88, 0x87, 0x77, 0x77, 0x66, 0x67, 0x76, 0x66, 0x77, 0x77, 0x78, 0x88, 0x99, 0xA0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xA, 0x99, 0x98, 0x88, 0x77, 0x77, 0x77, 0x80, 0x8, 0x77, 0x77, 0x77, 0x88, 0x89, 0x99, 0xA0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x7A, 0xA9, 0x99, 0x88, 0x88, 0x87, 0x87, 0x0, 0x0, 0x78, 0x78, 0x88, 0x88, 0x99, 0x9A, 0xA7, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xBB, 0xAA, 0xA9, 0x99, 0x88, 0x89, 0x0, 0x0, 0x0, 0x0, 0x98, 0x88, 0x99, 0x9A, 0xAA, 0xBB, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xCB, 0xBA, 0xAA, 0x99, 0x99, 0xA0, 0x0, 0x0, 0x0, 0x0, 0xA, 0x99, 0x99, 0xAA, 0xAB, 0xBC, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5, 0xBB, 0xBB, 0xAA, 0xAA, 0xA5, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5A, 0xAA, 0xAA, 0xBB, 0xBB, 0x50, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xC, 0xCC, 0xBB, 0xBB, 0xAB, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xBA, 0xBB, 0xBB, 0xCC, 0xC0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xD, 0xCC, 0xCB, 0xBB, 0x90, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x9, 0xBB, 0xBC, 0xCC, 0xD0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x3D, 0xDC, 0xCC, 0xC2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x2C, 0xCC, 0xCD, 0xD3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xBD, 0xDD, 0xDB, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xBD, 0xDD, 0xDB, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xEE, 0xED, 0x50, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5, 0xDE, 0xEE, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0xFE, 0xC0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xC, 0xEF, 0x10, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x6, 0xE6, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x6E, 0x60, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x50, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }};
