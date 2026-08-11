package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class LatestValueMailboxTest {
  @Test
  void returnsOnlyTheNewestPendingValue() {
    LatestValueMailbox<Integer> mailbox = new LatestValueMailbox<>();

    mailbox.offer(1);
    mailbox.offer(2);
    mailbox.offer(3);

    assertEquals(3, mailbox.takeLatest().orElseThrow());
    assertTrue(mailbox.takeLatest().isEmpty());
  }
}
