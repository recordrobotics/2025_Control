import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

public class ExampleTest {

    @BeforeEach
    public void setUp() {
        // Code to set up test environment
    }

    @AfterEach
    public void shutDown() throws Exception{
        // Code to clean up after tests
    }

    @Test
    public void testExample() {
        // Example test case
        assertTrue(true);
    }

    @Test
    public void testKill() {
        // test if kill works
    }
}