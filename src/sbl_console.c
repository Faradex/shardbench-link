/*
 * Turning a stream of printf writes into whole log lines.
 *
 * This lives here, in the platform-free half, for one reason: it is the part that got
 * the interleaving wrong twice, and in `port/` it cannot be tested. Here the "thread"
 * is an opaque pointer the caller supplies, so the host's test suite can reproduce any
 * interleaving it likes -- including the one that actually happened on the board.
 *
 * printf is not atomic and Mbed's console is unbuffered, so a single line reaches us in
 * several writes, sometimes one character at a time: `printf("[%s][%s]: ", "INFO", ...)`
 * can arrive as "[", "INFO", "]", "[", "MAIN", "]: ". Two threads doing that at once
 * into one buffer produce text belonging to neither, so each gets a buffer of its own.
 */
#include "sbl_internal.h"

void sbl_console_init(sbl_console *c)
{
    unsigned i;

    for (i = 0; i < SBL_CONSOLE_SLOTS; i++) {
        c->lines[i].owner = NULL;
        c->lines[i].fill = 0;
    }
}

/**
 * The level the text announces, and how many bytes of it to drop.
 *
 * Every line the SDK and the application produce starts "[LEVEL][MODULE]: ", so the
 * level is there to be had even though printf cannot carry one. The module tag stays:
 * it is the useful half.
 */
uint8_t sbl_console_level(const char *line, uint16_t len, uint16_t *skip)
{
    static const struct { const char *name; uint8_t len; uint8_t level; } LEVELS[] = {
        {"[ERROR]",   7, 1},
        {"[WARNING]", 9, 2},
        {"[INFO]",    6, 3},
        {"[DEBUG]",   7, 4}
    };
    unsigned i, c;

    *skip = 0;
    for (i = 0; i < sizeof(LEVELS) / sizeof(LEVELS[0]); i++) {
        if (len < LEVELS[i].len) {
            continue;
        }
        for (c = 0; c < LEVELS[i].len; c++) {
            if (line[c] != LEVELS[i].name[c]) {
                break;
            }
        }
        if (c == LEVELS[i].len) {
            *skip = LEVELS[i].len;
            return LEVELS[i].level;
        }
    }
    return SBL_LOG_INFO;
}

static void emit(sbl_console *c, sbl_line *line, sbl_console_fn out, void *user)
{
    uint16_t skip = 0;
    uint8_t level;

    (void)c;
    if (line->fill > 0) {
        level = sbl_console_level(line->text, line->fill, &skip);
        out(level, &line->text[skip], (uint16_t)(line->fill - skip), user);
    }
    line->fill = 0;
}

/**
 * The buffer this writer is building its line in.
 *
 * With every slot held by a writer mid-line, the fullest is sent early and reused: that
 * costs the tail of one line rather than refusing to log, and it can only happen with
 * more threads printing at once than there are slots.
 */
static sbl_line *slot_for(sbl_console *c, const void *owner,
                          sbl_console_fn out, void *user)
{
    unsigned i, fullest = 0;

    for (i = 0; i < SBL_CONSOLE_SLOTS; i++) {
        if (c->lines[i].owner == owner) {
            return &c->lines[i];
        }
    }
    for (i = 0; i < SBL_CONSOLE_SLOTS; i++) {
        if (c->lines[i].owner == NULL) {
            c->lines[i].owner = owner;
            return &c->lines[i];
        }
    }
    for (i = 1; i < SBL_CONSOLE_SLOTS; i++) {
        if (c->lines[i].fill > c->lines[fullest].fill) {
            fullest = i;
        }
    }
    emit(c, &c->lines[fullest], out, user);
    c->lines[fullest].owner = owner;
    return &c->lines[fullest];
}

void sbl_console_write(sbl_console *c, const void *owner,
                       const char *data, size_t len,
                       sbl_console_fn out, void *user)
{
    sbl_line *line = slot_for(c, owner, out, user);
    size_t i;

    for (i = 0; i < len; i++) {
        char ch = data[i];

        if (ch == '\r') {
            continue;                    /* the "\n\r" the debug macro appends */
        }
        if (ch == '\n') {
            emit(c, line, out, user);
            continue;                    /* the slot stays this writer's */
        }
        if (line->fill >= SBL_CONSOLE_LINE) {
            emit(c, line, out, user);    /* longer than the buffer: split it */
        }
        line->text[line->fill++] = ch;
    }
}

void sbl_console_reset(sbl_console *c)
{
    sbl_console_init(c);
}
